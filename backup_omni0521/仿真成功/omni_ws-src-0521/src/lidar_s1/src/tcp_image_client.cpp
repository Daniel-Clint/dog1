#if defined(ROS1_ENABLED)
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>  // ROS1 image_transport
#elif defined(ROS2_ENABLED)
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>  // ROS2 image_transport
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <stdexcept>
#include <fcntl.h>


#if defined(ROS2_ENABLED)
// ROS2类型别名
using ImageMsg = sensor_msgs::msg::Image;
using CompressedImageMsg = sensor_msgs::msg::CompressedImage;
using NodeSharedPtr = std::shared_ptr<rclcpp::Node>;
#else
// ROS1类型别名
using ImageMsg = sensor_msgs::Image;
using CompressedImageMsg = sensor_msgs::CompressedImage;
#endif

class TCPImageClient {
public:
#if defined(ROS2_ENABLED)
    // ROS2构造函数
    explicit TCPImageClient(NodeSharedPtr node, 
                           const std::string& server_ip, 
                           int server_port)
        : node_(node),
          it_(node),  // 初始化ImageTransport
          server_ip_(server_ip), 
          server_port_(server_port), 
          sockfd_(-1), 
          running_(true) {
        initPublishers();
        startThreads();
    }
#else
    // ROS1构造函数
    TCPImageClient(const std::string& server_ip, int server_port)
        : nh_("~"),  // 使用私有节点句柄
          it_(nh_),  // 初始化ImageTransport
          server_ip_(server_ip), server_port_(server_port), 
          sockfd_(-1), running_(true) {
        initPublishers();
        startThreads();
    }
#endif

    ~TCPImageClient() {
        running_ = false;
        if (connect_thread_.joinable()) connect_thread_.join();
        if (receive_thread_.joinable()) receive_thread_.join();
        disconnect();
    }

private:
    // 初始化发布器（ROS1/ROS2双版本）
#if defined(ROS2_ENABLED)
    void initPublishers() {
        // 使用ImageTransport创建原始图像发布器
        top_half_pub_ = it_.advertise("/fisheye/left/image_raw", 1);
        bottom_half_pub_ = it_.advertise("/fisheye/right/image_raw", 1);
    }
#else
    void initPublishers() {
        // 使用ImageTransport创建原始图像发布器
        top_half_pub_ = it_.advertise("/fisheye/left/image_raw", 1);
        bottom_half_pub_ = it_.advertise("/fisheye/right/image_raw", 1);
    }
#endif

    void startThreads() {
        connect_thread_ = std::thread(&TCPImageClient::reconnectLoop, this);
        receive_thread_ = std::thread(&TCPImageClient::receiveImages, this);
    }

    // 带超时的连接函数（通用）
    bool connectWithTimeout() {
        struct sockaddr_in serv_addr;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(server_port_);
        inet_pton(AF_INET, server_ip_.c_str(), &serv_addr.sin_addr);

        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) return false;

        // 设置非阻塞模式
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);

        // 尝试连接
        int conn_result = connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
        if (conn_result < 0 && errno != EINPROGRESS) {
            close(sock);
            return false;
        }

        // 设置连接超时 (3秒)
        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(sock, &write_fds);
        
        struct timeval tv {.tv_sec = 3, .tv_usec = 0};

        int ready = select(sock + 1, nullptr, &write_fds, nullptr, &tv);
        if (ready <= 0) {
            close(sock);
            return false;  // 超时或错误
        }

        // 检查连接状态
        int error = 0;
        socklen_t len = sizeof(error);
        getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len);
        if (error != 0) {
            close(sock);
            return false;
        }

        // 恢复阻塞模式
        fcntl(sock, F_SETFL, flags);
        
        {
            std::lock_guard<std::mutex> lock(sock_mutex_);
            sockfd_ = sock;
        }
        
        logInfo("Connected to %s:%d", server_ip_.c_str(), server_port_);
        return true;
    }

    void disconnect() {
        std::lock_guard<std::mutex> lock(sock_mutex_);
        if (sockfd_ != -1) {
            close(sockfd_);
            sockfd_ = -1;
            logWarn("Disconnected from server");
        }
    }

    bool isConnected() {
        std::lock_guard<std::mutex> lock(sock_mutex_);
        return sockfd_ != -1;
    }

    // 自动重连线程（通用）
    void reconnectLoop() {
        while (running_) {
            if (!isConnected()) {
                if (connectWithTimeout()) {
                    // 成功连接
                } else if (running_) {
                    logWarn("Connection failed, retrying in 1 second...");
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    // 带超时的读取函数（通用）
    bool readWithTimeout(void* buffer, size_t length, int timeout_sec) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        
        int sock;
        {
            std::lock_guard<std::mutex> lock(sock_mutex_);
            if (sockfd_ == -1) return false;
            sock = sockfd_;
            FD_SET(sock, &read_fds);
        }
        
        struct timeval tv {.tv_sec = timeout_sec, .tv_usec = 0};

        size_t bytes_received = 0;
        while (bytes_received < length && running_) {
            int ready = select(sock + 1, &read_fds, nullptr, nullptr, &tv);
            if (ready <= 0) return false;  // 超时或错误
            
            ssize_t n = recv(sock, static_cast<uint8_t*>(buffer) + bytes_received, 
                             length - bytes_received, 0);
            if (n <= 0) return false;  // 连接断开或错误
            
            bytes_received += n;
        }
        return true;
    }

    // 图像接收主循环（通用）
    void receiveImages() {
        while (running_) {
            if (!isConnected()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            try {
                // 1. 读取4字节的消息长度头
                uint32_t data_size;
                if (!readWithTimeout(&data_size, sizeof(data_size), 1)) {
                    disconnect();
                    continue;
                }
                
                // 网络字节序转换
                data_size = ntohl(data_size);
                
                // 2. 读取完整的消息数据
                std::vector<uint8_t> buffer(data_size);
                if (!readWithTimeout(buffer.data(), data_size, 3)) {
                    logWarn("Failed to receive complete image data");
                    disconnect();
                    continue;
                }

                // 3. 反序列化ROS消息
                CompressedImageMsg compressed_img;
                deserializeROSMessage(compressed_img, buffer);
                
                // 4. 转换为OpenCV图像
                cv::Mat cv_img = cv::imdecode(compressed_img.data, cv::IMREAD_COLOR);
                if (cv_img.empty()) {
                    logWarn("Failed to decode image");
                    continue;
                }
                
                // 5. 分割图像
                int height = cv_img.rows;
                int width = cv_img.cols;
                int split_point = height / 2;
                
                cv::Mat top_half = cv_img(cv::Rect(0, 0, width, split_point));
                cv::Mat bottom_half = cv_img(cv::Rect(0, split_point, width, height - split_point));
                
                // 6. 发布图像（使用通用接口）
                publishImage(top_half, compressed_img.header, "top_half");
                publishImage(bottom_half, compressed_img.header, "bottom_half");
            } catch (const std::exception& e) {
                logError("Error processing image: %s", e.what());
                disconnect();
            }
        }
    }

    // ROS消息反序列化（通用）
    void deserializeROSMessage(CompressedImageMsg& msg, 
                               const std::vector<uint8_t>& buffer) {
        const uint8_t* data = buffer.data();
        size_t offset = 0;
        
        // 解析序列号
        uint32_t seq;
        memcpy(&seq, data + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        
        // 解析时间戳
        int32_t sec, nsec;
        memcpy(&sec, data + offset, sizeof(int32_t));
        offset += sizeof(int32_t);
        memcpy(&nsec, data + offset, sizeof(int32_t));
        offset += sizeof(int32_t);
        
        // 解析坐标系
        uint32_t frame_id_len;
        memcpy(&frame_id_len, data + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        msg.header.frame_id.assign(reinterpret_cast<const char*>(data + offset), frame_id_len);
        offset += frame_id_len;
        
        // 解析格式
        uint32_t format_len;
        memcpy(&format_len, data + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        msg.format.assign(reinterpret_cast<const char*>(data + offset), format_len);
        offset += format_len;
        
        // 解析图像数据
        uint32_t data_len;
        memcpy(&data_len, data + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        msg.data.assign(data + offset, data + offset + data_len);
        
        // 设置时间戳（ROS1/ROS2兼容）
#if defined(ROS2_ENABLED)
        msg.header.stamp.sec = sec;
        msg.header.stamp.nanosec = nsec;
#else
        msg.header.stamp.sec = sec;
        msg.header.stamp.nsec = nsec;
        msg.header.seq = seq;
#endif
    }

    // 日志工具函数（兼容ROS1/ROS2）
    void logInfo(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);
        
#if defined(ROS2_ENABLED)
        RCLCPP_INFO(node_->get_logger(), "%s", buffer);
#else
        ROS_INFO("%s", buffer);
#endif
    }
    
    void logWarn(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);
        
#if defined(ROS2_ENABLED)
        RCLCPP_WARN(node_->get_logger(), "%s", buffer);
#else
        ROS_WARN("%s", buffer);
#endif
    }
    
    void logError(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);
        
#if defined(ROS2_ENABLED)
        RCLCPP_ERROR(node_->get_logger(), "%s", buffer);
#else
        ROS_ERROR("%s", buffer);
#endif
    }

    // 发布图像（兼容ROS1/ROS2）
    void publishImage(const cv::Mat& image, 
                     const auto& header,  // 使用auto处理ROS1/ROS2不同的header类型
                     const std::string& name) {
        try {
            cv_bridge::CvImage cv_image;
            
            // 设置header（兼容ROS1/ROS2）
#if defined(ROS2_ENABLED)
            cv_image.header.stamp = header.stamp;
            cv_image.header.frame_id = header.frame_id;
#else
            cv_image.header.stamp = header.stamp;
            cv_image.header.frame_id = header.frame_id;
            cv_image.header.seq = header.seq;
#endif
            
            cv_image.encoding = sensor_msgs::image_encodings::BGR8;
            cv_image.image = image;
            
            // 发布图像（使用image_transport）
            auto img_msg = cv_image.toImageMsg();
            
            if (name == "top_half") {
                top_half_pub_.publish(img_msg);
            } else {
                bottom_half_pub_.publish(img_msg);
            }
        } catch (const std::exception& e) {
            logError("Error publishing %s image: %s", name.c_str(), e.what());
        }
    }
    
    // 成员变量
    std::string server_ip_;
    int server_port_;
    std::atomic<int> sockfd_;
    std::atomic<bool> running_;
    
#if defined(ROS2_ENABLED)
    // ROS2发布器
    NodeSharedPtr node_;
    image_transport::ImageTransport it_;  // ROS2 ImageTransport
    image_transport::Publisher top_half_pub_;
    image_transport::Publisher bottom_half_pub_;
#else
    // ROS1发布器
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;  // ROS1 ImageTransport
    image_transport::Publisher top_half_pub_;
    image_transport::Publisher bottom_half_pub_;
#endif
    
    std::mutex sock_mutex_;
    std::thread connect_thread_;
    std::thread receive_thread_;
};

// 主函数（ROS1/ROS2双版本）
#if defined(ROS2_ENABLED)
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("tcp_image_client");
    
    // 从参数服务器获取IP和端口
    std::string server_ip = node->declare_parameter("server_ip", "192.168.1.2");
    int server_port = node->declare_parameter("server_port", 8888);
    
    RCLCPP_INFO(node->get_logger(), "Starting TCP image client. Server: %s:%d", 
                server_ip.c_str(), server_port);
    
    try {
        TCPImageClient client(node, server_ip, server_port);
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Initialization failed: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
#else
int main(int argc, char** argv) {
    ros::init(argc, argv, "tcp_image_client");
    
    // 从参数服务器获取IP和端口
    ros::NodeHandle nh("~");
    std::string server_ip;
    int server_port;
    
    nh.param<std::string>("server_ip", server_ip, "192.168.1.2");
    nh.param<int>("server_port", server_port, 8888);
    
    ROS_INFO("Starting TCP image client. Server: %s:%d", server_ip.c_str(), server_port);
    
    try {
        TCPImageClient client(server_ip, server_port);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("Initialization failed: %s", e.what());
        return 1;
    }
    
    return 0;
}
#endif