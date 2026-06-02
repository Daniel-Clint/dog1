/**
 * @file edge_agent_node.cpp
 * @brief 机器狗边缘端代理节点
 *
 * 说明：
 * 1. 当前文件优先完成“协议层”和“任务上下文管理”，便于与后端接口文档对齐。
 * 2. 真实 WebSocket 收发仍保留为可替换的传输层占位，避免引入额外依赖后破坏现有工程编译。
 * 3. 为便于本地联调，节点额外提供两个 ROS Topic：
 *    - /edge_agent/incoming_text:   注入服务端下发的 JSON 文本
 *    - /edge_agent/outgoing_text:   观察节点即将发送给服务端的 JSON 文本
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <actionlib_msgs/GoalID.h>
#include <omni_stitch_capture/StitchedCapture.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

#include <curl/curl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <unordered_set>
#include <string>
#include <thread>
#include <vector>

#include "nlohmann/json.hpp"
#include "lz4.h"

using json = nlohmann::json;
namespace asio = boost::asio;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
using tcp = asio::ip::tcp;

namespace
{

std::string base64Encode(const uint8_t* data, size_t len)
{
    static const char kTable[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    std::string encoded;
    encoded.reserve(((len + 2) / 3) * 4);

    for (size_t i = 0; i < len; i += 3) {
        const uint32_t octet_a = data[i];
        const uint32_t octet_b = (i + 1 < len) ? data[i + 1] : 0;
        const uint32_t octet_c = (i + 2 < len) ? data[i + 2] : 0;
        const uint32_t triple = (octet_a << 16) | (octet_b << 8) | octet_c;

        encoded.push_back(kTable[(triple >> 18) & 0x3F]);
        encoded.push_back(kTable[(triple >> 12) & 0x3F]);
        encoded.push_back((i + 1 < len) ? kTable[(triple >> 6) & 0x3F] : '=');
        encoded.push_back((i + 2 < len) ? kTable[triple & 0x3F] : '=');
    }

    return encoded;
}

std::string base64Encode(const std::vector<uint8_t>& data)
{
    if (data.empty()) {
        return "";
    }
    return base64Encode(data.data(), data.size());
}

std::string base64Encode(const std::vector<char>& data)
{
    if (data.empty()) {
        return "";
    }
    return base64Encode(reinterpret_cast<const uint8_t*>(data.data()), data.size());
}

bool fileExists(const std::string& path)
{
    std::ifstream file(path.c_str(), std::ios::binary);
    return file.good();
}

size_t curlWriteCallback(void* contents, size_t size, size_t nmemb, void* userp)
{
    const size_t total_size = size * nmemb;
    std::string* response = static_cast<std::string*>(userp);
    response->append(static_cast<char*>(contents), total_size);
    return total_size;
}

bool encodeGrayImageToJpegBase64(const cv::Mat& gray_image,
                                 int jpeg_quality,
                                 std::string& encoded_payload)
{
    if (gray_image.empty()) {
        return false;
    }

    try {
        std::vector<uint8_t> jpeg_buffer;
        std::vector<int> params;
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(jpeg_quality);
        if (!cv::imencode(".jpg", gray_image, jpeg_buffer, params) || jpeg_buffer.empty()) {
            return false;
        }

        encoded_payload = base64Encode(jpeg_buffer);
        return true;
    } catch (const cv::Exception&) {
        return false;
    }
}

double finiteOrDefault(double value, double fallback)
{
    return std::isfinite(value) ? value : fallback;
}

void quaternionToEuler(double x, double y, double z, double w,
                       double& roll, double& pitch, double& yaw)
{
    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(M_PI / 2.0, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

bool parseYawRadians(const json& waypoint, double& yaw_radians)
{
    if (!waypoint.contains("yaw") || waypoint["yaw"].is_null()) {
        yaw_radians = 0.0;
        return false;
    }

    if (waypoint["yaw"].is_number()) {
        yaw_radians = waypoint["yaw"].get<double>();
        return true;
    }

    yaw_radians = 0.0;
    return false;
}

void fillYawOrientation(double yaw_radians, geometry_msgs::Quaternion& q)
{
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw_radians * 0.5);
    q.w = std::cos(yaw_radians * 0.5);
}

struct WebSocketEndpoint
{
    std::string scheme;
    std::string host;
    std::string port;
    std::string target;
};

bool parseWebSocketUrl(const std::string& url, WebSocketEndpoint& endpoint)
{
    const std::string prefix = "ws://";
    if (url.compare(0, prefix.size(), prefix) != 0) {
        return false;
    }

    endpoint.scheme = "ws";
    const std::string rest = url.substr(prefix.size());
    const std::string::size_type slash_pos = rest.find('/');
    const std::string host_port = (slash_pos == std::string::npos) ? rest : rest.substr(0, slash_pos);
    endpoint.target = (slash_pos == std::string::npos) ? "/" : rest.substr(slash_pos);

    const std::string::size_type colon_pos = host_port.rfind(':');
    if (colon_pos == std::string::npos || colon_pos == 0 || colon_pos == host_port.size() - 1) {
        return false;
    }

    endpoint.host = host_port.substr(0, colon_pos);
    endpoint.port = host_port.substr(colon_pos + 1);
    return !endpoint.host.empty() && !endpoint.port.empty() && !endpoint.target.empty();
}

struct SimplePoint
{
    float x;
    float y;
    float z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct VoxelKey
{
    int x;
    int y;
    int z;

    bool operator==(const VoxelKey& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash
{
    std::size_t operator()(const VoxelKey& key) const
    {
        std::size_t seed = 0;
        seed ^= std::hash<int>()(key.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

} // namespace

class EdgeAgent
{
public:
    explicit EdgeAgent(ros::NodeHandle& nh)
        : nh_(nh),
          ws_connected_(false),
          registered_(false),
          stop_ws_thread_(false),
          voxel_size_(0.1),
          jpeg_quality_(70),
          heartbeat_interval_sec_(5.0),
          status_interval_sec_(2.0),
          default_battery_level_(100.0),
          default_temperature_(35.0),
          default_voltage_(28.8),
          default_height_(0.30),
          default_remain_mile_(10.0),
          default_cpu_usage_(12.0),
          motion_state_(0),
          gait_(0x1001),
          control_usage_mode_(1),
          direction_(0),
          sleep_state_(0),
          version_(0),
          hes_state_(0),
          ooa_state_(0),
          charge_state_(0),
          power_management_(0),
          lidar_front_(1),
          lidar_back_(1),
          gps_enabled_(1),
          video_front_(1),
          video_back_(1),
          led_fill_front_(1),
          led_fill_back_(1),
          gray_map_resolution_(0.10),
          gray_map_z_min_(-std::numeric_limits<double>::infinity()),
          gray_map_z_max_(std::numeric_limits<double>::infinity()),
          max_gray_map_pixels_(4000000),
          enable_local_cloud_(true),
          enable_video_frame_(true),
          publish_gray_submap_(true),
          auto_ack_supported_commands_(true),
          auto_upload_map_on_save_(true),
          mapping_active_(false),
          save_map_upload_delay_sec_(3.0),
          save_map_upload_retry_count_(3),
          latest_roll_(0.0),
          latest_pitch_(0.0),
          latest_yaw_(0.0),
          latest_omega_z_(0.0),
          latest_linear_x_(0.0),
          latest_linear_y_(0.0),
          latest_height_(0.30)
    {
        readParameters();

        ROS_INFO("EdgeAgent started, robot_id=%s", robot_id_.c_str());
        ROS_INFO("WebSocket URL: %s", server_url_.c_str());
        ROS_INFO("HTTP upload template: %s", upload_url_template_.c_str());

        pose_sub_ = nh_.subscribe("/aft_mapped_to_init", 10, &EdgeAgent::poseCallback, this);
        cloud_sub_ = nh_.subscribe("/cloud_registered", 3, &EdgeAgent::pointCloudCallback, this);
        image_sub_ = nh_.subscribe(camera_topic_, 3, &EdgeAgent::imageCallback, this);
        if (enable_stitch_capture_) {
            stitch_capture_sub_ = nh_.subscribe(
                stitch_capture_topic_, 3, &EdgeAgent::stitchCaptureCallback, this);
        }
        incoming_text_sub_ = nh_.subscribe("/edge_agent/incoming_text", 20, &EdgeAgent::incomingTextCallback, this);
        alert_sub_ = nh_.subscribe("/edge_agent/alert", 20, &EdgeAgent::alertTextCallback, this);

        outgoing_text_pub_ = nh_.advertise<std_msgs::String>("/edge_agent/outgoing_text", 20);
        navigation_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        navigation_cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);

        heartbeat_timer_ = nh_.createTimer(
            ros::Duration(heartbeat_interval_sec_),
            &EdgeAgent::heartbeatTimerCallback,
            this);
        status_timer_ = nh_.createTimer(
            ros::Duration(status_interval_sec_),
            &EdgeAgent::statusTimerCallback,
            this);

        curl_global_init(CURL_GLOBAL_DEFAULT);
        connectToServer();
    }

    ~EdgeAgent()
    {
        stopWebSocketThread();
        curl_global_cleanup();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber image_sub_;
    ros::Subscriber stitch_capture_sub_;
    ros::Subscriber incoming_text_sub_;
    ros::Subscriber alert_sub_;
    ros::Publisher outgoing_text_pub_;
    ros::Publisher navigation_goal_pub_;
    ros::Publisher navigation_cancel_pub_;
    ros::Timer heartbeat_timer_;
    ros::Timer status_timer_;

    std::mutex state_mutex_;
    std::mutex map_mutex_;
    std::mutex ws_queue_mutex_;
    std::condition_variable ws_queue_cv_;
    std::deque<std::string> reliable_ws_queue_;
    std::deque<std::string> status_ws_window_;
    std::deque<std::string> pose_ws_window_;
    std::deque<std::string> panorama_pose_ws_window_;
    std::deque<std::string> video_frame_ws_window_;
    std::deque<std::string> local_cloud_ws_window_;
    std::deque<std::string> global_submap_ws_window_;
    std::thread ws_thread_;

    std::string server_url_;
    std::string upload_url_template_;
    std::string robot_id_;
    std::string current_task_id_;
    std::string mode_;
    std::string camera_id_;
    std::string camera_topic_;
    std::string stitch_capture_topic_;
    std::string navigation_frame_id_;
    std::string global_map_file_path_;
    std::string active_mapping_task_id_;
    std::atomic<bool> ws_connected_;
    std::atomic<bool> registered_;
    std::atomic<bool> stop_ws_thread_;
    double voxel_size_;
    int jpeg_quality_;
    double heartbeat_interval_sec_;
    double status_interval_sec_;
    double default_battery_level_;
    double default_temperature_;
    double default_voltage_;
    double default_height_;
    double default_remain_mile_;
    double default_cpu_usage_;
    int motion_state_;
    int gait_;
    int control_usage_mode_;
    int direction_;
    int sleep_state_;
    int version_;
    int hes_state_;
    int ooa_state_;
    int charge_state_;
    int power_management_;
    int lidar_front_;
    int lidar_back_;
    int gps_enabled_;
    int video_front_;
    int video_back_;
    int led_fill_front_;
    int led_fill_back_;
    double gray_map_resolution_;
    double gray_map_z_min_;
    double gray_map_z_max_;
    int max_gray_map_pixels_;
    static constexpr std::size_t kStatusWindowSize = 3;
    static constexpr std::size_t kPoseWindowSize = 3;
    static constexpr std::size_t kPanoramaPoseWindowSize = 2;
    static constexpr std::size_t kVideoFrameWindowSize = 2;
    static constexpr std::size_t kLocalCloudWindowSize = 2;
    static constexpr std::size_t kGlobalSubmapWindowSize = 2;
    bool enable_local_cloud_;
    bool enable_video_frame_;
    bool enable_stitch_capture_;
    bool publish_gray_submap_;
    bool auto_ack_supported_commands_;
    bool auto_upload_map_on_save_;
    bool mapping_active_;
    double save_map_upload_delay_sec_;
    int save_map_upload_retry_count_;
    double latest_roll_;
    double latest_pitch_;
    double latest_yaw_;
    double latest_omega_z_;
    double latest_linear_x_;
    double latest_linear_y_;
    double latest_height_;
    std::vector<SimplePoint> accumulated_map_points_;
    std::unordered_set<VoxelKey, VoxelKeyHash> accumulated_map_voxels_;

    void readParameters()
    {
        nh_.param<std::string>("server_url", server_url_, "ws://172.18.10.9:8599/ws/edge");
        nh_.param<std::string>("upload_url_template", upload_url_template_,
                               "http://172.18.10.9:8599/api/v1/edge/tasks/{taskId}/files");
        nh_.param<std::string>("robot_id", robot_id_, "dog_01");
        nh_.param<double>("pointcloud_voxel_size", voxel_size_, 0.1);
        nh_.param<int>("image_jpeg_quality", jpeg_quality_, 70);
        nh_.param<double>("heartbeat_interval_sec", heartbeat_interval_sec_, 5.0);
        nh_.param<double>("status_interval_sec", status_interval_sec_, 2.0);
        nh_.param<double>("default_battery_level", default_battery_level_, 100.0);
        nh_.param<double>("default_temperature", default_temperature_, 35.0);
        nh_.param<double>("default_voltage", default_voltage_, 28.8);
        nh_.param<double>("default_height", default_height_, 0.30);
        nh_.param<double>("default_remain_mile", default_remain_mile_, 10.0);
        nh_.param<double>("default_cpu_usage", default_cpu_usage_, 12.0);
        nh_.param<int>("motion_state", motion_state_, 0);
        nh_.param<int>("gait", gait_, 0x1001);
        nh_.param<int>("control_usage_mode", control_usage_mode_, 1);
        nh_.param<int>("direction", direction_, 0);
        nh_.param<int>("sleep_state", sleep_state_, 0);
        nh_.param<int>("version", version_, 0);
        nh_.param<int>("hes", hes_state_, 0);
        nh_.param<int>("ooa", ooa_state_, 0);
        nh_.param<int>("charge", charge_state_, 0);
        nh_.param<int>("power_management", power_management_, 0);
        nh_.param<int>("lidar_front", lidar_front_, 1);
        nh_.param<int>("lidar_back", lidar_back_, 1);
        nh_.param<int>("gps_enabled", gps_enabled_, 1);
        nh_.param<int>("video_front", video_front_, 1);
        nh_.param<int>("video_back", video_back_, 1);
        nh_.param<int>("led_fill_front", led_fill_front_, 1);
        nh_.param<int>("led_fill_back", led_fill_back_, 1);
        nh_.param<double>("gray_map_resolution", gray_map_resolution_, 0.10);
        nh_.param<double>("gray_map_z_min", gray_map_z_min_, -std::numeric_limits<double>::infinity());
        nh_.param<double>("gray_map_z_max", gray_map_z_max_, std::numeric_limits<double>::infinity());
        nh_.param<int>("max_gray_map_pixels", max_gray_map_pixels_, 4000000);
        nh_.param<bool>("enable_local_cloud", enable_local_cloud_, true);
        nh_.param<bool>("enable_video_frame", enable_video_frame_, true);
        nh_.param<bool>("enable_stitch_capture", enable_stitch_capture_, false);
        nh_.param<bool>("publish_gray_submap", publish_gray_submap_, true);
        nh_.param<std::string>("mode", mode_, "idle");
        nh_.param<std::string>("camera_id", camera_id_, "front");
        nh_.param<std::string>("camera_topic", camera_topic_, "/rgb_img");
        nh_.param<std::string>("stitch_capture_topic", stitch_capture_topic_, "/omni_stitch_capture/capture");
        nh_.param<std::string>("navigation_frame_id", navigation_frame_id_, "map");
        nh_.param<std::string>("global_map_file_path", global_map_file_path_, "");
        nh_.param<bool>("auto_ack_supported_commands", auto_ack_supported_commands_, true);
        nh_.param<bool>("auto_upload_map_on_save", auto_upload_map_on_save_, true);
        nh_.param<double>("save_map_upload_delay_sec", save_map_upload_delay_sec_, 3.0);
        nh_.param<int>("save_map_upload_retry_count", save_map_upload_retry_count_, 3);
    }

    bool canSendTelemetry() const
    {
        return ws_connected_.load() && registered_.load();
    }

    bool canSendMappingTelemetry() const
    {
        return canSendTelemetry() && mapping_active_ && !current_task_id_.empty();
    }

    json buildBaseEvent(const std::string& event_name, bool attach_task_id = true)
    {
        json j;
        j["event"] = event_name;
        j["robotCode"] = robot_id_;

        if (attach_task_id) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!current_task_id_.empty()) {
                j["taskId"] = current_task_id_;
            }
        }

        return j;
    }

    void connectToServer()
    {
        if (ws_thread_.joinable()) {
            return;
        }
        stop_ws_thread_.store(false);
        ws_thread_ = std::thread(&EdgeAgent::webSocketLoop, this);
    }

    void sendText(const json& payload)
    {
        const std::string text = payload.dump();

        std_msgs::String msg;
        msg.data = text;
        outgoing_text_pub_.publish(msg);

        enqueueWebSocketMessage(payload, text);
        ROS_DEBUG_STREAM("EdgeAgent 发送: " << text);
    }

    void stopWebSocketThread()
    {
        stop_ws_thread_.store(true);
        ws_queue_cv_.notify_all();
        if (ws_thread_.joinable()) {
            ws_thread_.join();
        }
    }

    void enqueueWebSocketMessage(const json& payload, const std::string& text)
    {
        std::lock_guard<std::mutex> lock(ws_queue_mutex_);
        const std::string event_name = payload.value("event", "");

        if (event_name == "status") {
            pushSlidingWindowMessage(status_ws_window_, text, kStatusWindowSize);
        } else if (event_name == "pose") {
            bool has_panorama = false;
            if (payload.contains("data") && payload["data"].is_object()) {
                const auto& data = payload["data"];
                has_panorama = data.contains("panorama") &&
                               data["panorama"].is_string() &&
                               !data["panorama"].get<std::string>().empty();
            }

            if (has_panorama) {
                pushSlidingWindowMessage(panorama_pose_ws_window_, text, kPanoramaPoseWindowSize);
            } else {
                pushSlidingWindowMessage(pose_ws_window_, text, kPoseWindowSize);
            }
        } else if (event_name == "video.frame") {
            pushSlidingWindowMessage(video_frame_ws_window_, text, kVideoFrameWindowSize);
        } else if (event_name == "localCloud") {
            pushSlidingWindowMessage(local_cloud_ws_window_, text, kLocalCloudWindowSize);
        } else if (event_name == "globalSubmap") {
            pushSlidingWindowMessage(global_submap_ws_window_, text, kGlobalSubmapWindowSize);
        } else {
            reliable_ws_queue_.push_back(text);
        }

        ws_queue_cv_.notify_one();
    }

    void clearPendingWebSocketMessages()
    {
        std::lock_guard<std::mutex> lock(ws_queue_mutex_);
        reliable_ws_queue_.clear();
        status_ws_window_.clear();
        pose_ws_window_.clear();
        panorama_pose_ws_window_.clear();
        video_frame_ws_window_.clear();
        local_cloud_ws_window_.clear();
        global_submap_ws_window_.clear();
    }

    void webSocketLoop()
    {
        WebSocketEndpoint endpoint;
        if (!parseWebSocketUrl(server_url_, endpoint)) {
            ROS_ERROR("Invalid WebSocket URL, only ws://host:port/path is supported: %s", server_url_.c_str());
            return;
        }

        while (ros::ok() && !stop_ws_thread_.load()) {
            try {
                runSingleWebSocketSession(endpoint);
            } catch (const std::exception& e) {
                ws_connected_.store(false);
                registered_.store(false);
                ROS_ERROR("WebSocket session exception: %s", e.what());
            }

            if (stop_ws_thread_.load() || !ros::ok()) {
                break;
            }

            ROS_WARN("WebSocket disconnected, retrying in 5 seconds...");
            for (int i = 0; i < 50 && ros::ok() && !stop_ws_thread_.load(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    void runSingleWebSocketSession(const WebSocketEndpoint& endpoint)
    {
        asio::io_context io_context;
        tcp::resolver resolver(io_context);
        websocket::stream<tcp::socket> ws(io_context);

        auto const results = resolver.resolve(endpoint.host, endpoint.port);
        asio::connect(ws.next_layer(), results.begin(), results.end());

        ws.set_option(websocket::stream_base::timeout::suggested(beast::role_type::client));
        ws.set_option(websocket::stream_base::decorator(
            [](websocket::request_type& req) {
                req.set(beast::http::field::user_agent, std::string("omni-livo-edge-agent"));
            }));

        ws.handshake(endpoint.host + ":" + endpoint.port, endpoint.target);
        ws_connected_.store(true);
        registered_.store(false);
        clearPendingWebSocketMessages();
        ROS_INFO("WebSocket connected to %s", server_url_.c_str());

        sendRegister();
        runSessionLoop(ws);

        beast::error_code ec;
        if (ws.is_open()) {
            ws.close(websocket::close_code::normal, ec);
        }

        ws_connected_.store(false);
        registered_.store(false);
    }

    void runSessionLoop(websocket::stream<tcp::socket>& ws)
    {
        while (ros::ok() && !stop_ws_thread_.load() && ws_connected_.load()) {
            try {
                beast::error_code ec;
                const std::size_t available_bytes = ws.next_layer().available(ec);

                if (ec) {
                    ROS_WARN("Failed to query socket available bytes: %s", ec.message().c_str());
                    break;
                }

                if (available_bytes > 0) {
                    beast::flat_buffer buffer;
                    ws.read(buffer, ec);

                    if (!ec) {
                        const std::string text = beast::buffers_to_string(buffer.data());
                        handleServerMessage(text);
                    } else if (ec == websocket::error::closed) {
                        ROS_WARN("WebSocket closed by server.");
                        break;
                    } else {
                        ROS_WARN("WebSocket session loop stopped: %s", ec.message().c_str());
                        break;
                    }
                }

                flushPendingMessages(ws);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            } catch (const std::exception& e) {
                ROS_WARN("WebSocket session loop exception: %s", e.what());
                break;
            }
        }

        ws_connected_.store(false);
        registered_.store(false);
        ws_queue_cv_.notify_all();
    }

    bool popReliableWebSocketMessage(std::string& text)
    {
        std::lock_guard<std::mutex> lock(ws_queue_mutex_);
        if (reliable_ws_queue_.empty()) {
            return false;
        }

        text = reliable_ws_queue_.front();
        reliable_ws_queue_.pop_front();
        return true;
    }

    void pushSlidingWindowMessage(std::deque<std::string>& window,
                                  const std::string& text,
                                  std::size_t max_size)
    {
        window.push_back(text);
        while (window.size() > max_size) {
            window.pop_front();
        }
    }

    bool popWindowWebSocketMessage(std::string& text, std::deque<std::string>& window)
    {
        std::lock_guard<std::mutex> lock(ws_queue_mutex_);
        if (window.empty()) {
            return false;
        }

        text = window.front();
        window.pop_front();
        return true;
    }

    void flushPendingMessages(websocket::stream<tcp::socket>& ws)
    {
        std::string text;

        while (ws_connected_.load() && !stop_ws_thread_.load() && popReliableWebSocketMessage(text)) {
            ws.write(asio::buffer(text));
        }

        if (ws_connected_.load() && !stop_ws_thread_.load() &&
            popWindowWebSocketMessage(text, status_ws_window_)) {
            ws.write(asio::buffer(text));
        }

        if (ws_connected_.load() && !stop_ws_thread_.load() &&
            popWindowWebSocketMessage(text, pose_ws_window_)) {
            ws.write(asio::buffer(text));
        }

        if (ws_connected_.load() && !stop_ws_thread_.load() &&
            popWindowWebSocketMessage(text, panorama_pose_ws_window_)) {
            ws.write(asio::buffer(text));
        }

        if (ws_connected_.load() && !stop_ws_thread_.load() &&
            popWindowWebSocketMessage(text, video_frame_ws_window_)) {
            ws.write(asio::buffer(text));
        }

        if (ws_connected_.load() && !stop_ws_thread_.load() &&
            popWindowWebSocketMessage(text, local_cloud_ws_window_)) {
            ws.write(asio::buffer(text));
        }

        if (ws_connected_.load() && !stop_ws_thread_.load() &&
            popWindowWebSocketMessage(text, global_submap_ws_window_)) {
            ws.write(asio::buffer(text));
        }
    }

    void sendRegister()
    {
        json j;
        j["event"] = "register";
        j["robotCode"] = robot_id_;
        sendText(j);
    }

    void sendHeartbeat()
    {
        if (!ws_connected_) {
            return;
        }

        json j;
        j["event"] = "heartbeat";
        j["robotCode"] = robot_id_;
        sendText(j); 
    }

    void sendStatus()
    {
        if (!canSendMappingTelemetry()) {
            return;
        }

        json cpu_status = {
            {"temperature", default_temperature_},
            {"frequencyInt", default_cpu_usage_},
            {"frequencyApp", default_cpu_usage_}
        };

        json j = buildBaseEvent("status");
        j["data"] = {
            {"motionState", motion_state_},
            {"gait", gait_},
            {"controlUsageMode", control_usage_mode_},
            {"direction", direction_},
            {"sleep", sleep_state_},
            {"version", version_},
            {"hes", hes_state_},
            {"ooa", ooa_state_},
            {"charge", charge_state_},
            {"powerManagement", power_management_},
            {"batteryLevelLeft", default_battery_level_},
            {"batteryLevelRight", default_battery_level_},
            {"voltageLeft", default_voltage_},
            {"voltageRight", default_voltage_},
            {"batteryTemperatureLeft", default_temperature_},
            {"batteryTemperatureRight", default_temperature_},
            {"chargeLeft", false},
            {"chargeRight", false},
            {"roll", latest_roll_},
            {"pitch", latest_pitch_},
            {"yaw", latest_yaw_},
            {"omegaZ", latest_omega_z_},
            {"linearX", latest_linear_x_},
            {"linearY", latest_linear_y_},
            {"height", latest_height_},
            {"remainMile", default_remain_mile_},
            {"lidar", {
                {"front", lidar_front_},
                {"back", lidar_back_}
            }},
            {"gps", gps_enabled_},
            {"video", {
                {"front", video_front_},
                {"back", video_back_}
            }},
            {"led", {
                {"fill", {
                    {"front", led_fill_front_},
                    {"back", led_fill_back_}
                }}
            }},
            {"errorList", json::array()},
            {"cpu", {
                {"aos", cpu_status},
                {"nos", cpu_status},
                {"gos", cpu_status}
            }}
        };
        sendText(j);
    }

    void sendAlert(const std::string& alert_code,
                   const std::string& level,
                   const std::string& message)
    {
        if (!canSendTelemetry()) {
            return;
        }

        json j = buildBaseEvent("alert");
        j["data"] = {
            {"alertCode", alert_code},
            {"level", level},
            {"message", message}
        };
        sendText(j);
    }

    void sendCommandAck(const std::string& command_id,
                        const std::string& task_id,
                        const std::string& status)
    {
        json j;
        j["event"] = "command_ack";
        j["commandId"] = command_id;
        if (!task_id.empty()) {
            j["taskId"] = task_id;
        }
        j["status"] = status;
        sendText(j);
    }

    void sendGlobalSubmapGrayImage(const std::vector<SimplePoint>& cloud,
                                   const std_msgs::Header& header)
    {
        if (!publish_gray_submap_ || cloud.empty()) {
            return;
        }

        std::vector<SimplePoint> filtered_cloud;
        filtered_cloud.reserve(cloud.size());

        double x_min = std::numeric_limits<double>::infinity();
        double x_max = -std::numeric_limits<double>::infinity();
        double y_min = std::numeric_limits<double>::infinity();
        double y_max = -std::numeric_limits<double>::infinity();

        // 参考 2D_map.py：先按高度范围过滤，再直接投影到 XY 平面生成灰度图。
        for (const auto& pt : cloud) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }
            if (pt.z < gray_map_z_min_ || pt.z > gray_map_z_max_) {
                continue;
            }

            filtered_cloud.push_back(pt);
            x_min = std::min(x_min, static_cast<double>(pt.x));
            x_max = std::max(x_max, static_cast<double>(pt.x));
            y_min = std::min(y_min, static_cast<double>(pt.y));
            y_max = std::max(y_max, static_cast<double>(pt.y));
        }

        if (filtered_cloud.empty()) {
            return;
        }

        const int width = std::max(1, static_cast<int>((x_max - x_min) / gray_map_resolution_) + 1);
        const int height = std::max(1, static_cast<int>((y_max - y_min) / gray_map_resolution_) + 1);

        if (static_cast<int64_t>(width) * static_cast<int64_t>(height) > static_cast<int64_t>(max_gray_map_pixels_)) {
            ROS_WARN("Skipping grayscale submap: projected image is too large (%d x %d).", width, height);
            return;
        }

        cv::Mat gray_map(height, width, CV_8UC1, cv::Scalar(255));

        for (const auto& pt : filtered_cloud) {
            int pixel_x = static_cast<int>((pt.x - x_min) / gray_map_resolution_);
            int pixel_y = static_cast<int>((pt.y - y_min) / gray_map_resolution_);
            pixel_x = std::max(0, std::min(width - 1, pixel_x));
            pixel_y = std::max(0, std::min(height - 1, pixel_y));

            // 与 2D_map.py 保持一致：有点区域置黑，并翻转 Y 轴到图像坐标系。
            gray_map.at<uint8_t>(height - 1 - pixel_y, pixel_x) = 0;
        }

        std::string payload_base64;
        if (!encodeGrayImageToJpegBase64(gray_map, jpeg_quality_, payload_base64)) {
            ROS_WARN("Failed to encode 2D grayscale submap as JPEG.");
            return;
        }

        json j = buildBaseEvent("globalSubmap");
        j["data"] = {
            {"source", "omni_livo_incremental_cloud"},
            {"frameId", header.frame_id},
            {"width", gray_map.cols},
            {"height", gray_map.rows},
            {"resolution", gray_map_resolution_},
            {"zMin", gray_map_z_min_},
            {"zMax", gray_map_z_max_},
            {"format", "jpeg"},
            {"payload", payload_base64}
        };
        sendText(j);
    }

    bool buildGrayMapImage(const std::vector<SimplePoint>& cloud,
                           cv::Mat& gray_map,
                           double& x_min,
                           double& y_min) const
    {
        if (cloud.empty()) {
            return false;
        }

        std::vector<SimplePoint> filtered_cloud;
        filtered_cloud.reserve(cloud.size());

        double local_x_min = std::numeric_limits<double>::infinity();
        double x_max = -std::numeric_limits<double>::infinity();
        double local_y_min = std::numeric_limits<double>::infinity();
        double y_max = -std::numeric_limits<double>::infinity();

        for (const auto& pt : cloud) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }
            if (pt.z < gray_map_z_min_ || pt.z > gray_map_z_max_) {
                continue;
            }

            filtered_cloud.push_back(pt);
            local_x_min = std::min(local_x_min, static_cast<double>(pt.x));
            x_max = std::max(x_max, static_cast<double>(pt.x));
            local_y_min = std::min(local_y_min, static_cast<double>(pt.y));
            y_max = std::max(y_max, static_cast<double>(pt.y));
        }

        if (filtered_cloud.empty()) {
            return false;
        }

        const int width = std::max(1, static_cast<int>((x_max - local_x_min) / gray_map_resolution_) + 1);
        const int height = std::max(1, static_cast<int>((y_max - local_y_min) / gray_map_resolution_) + 1);

        if (static_cast<int64_t>(width) * static_cast<int64_t>(height) > static_cast<int64_t>(max_gray_map_pixels_)) {
            return false;
        }

        gray_map = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
        for (const auto& pt : filtered_cloud) {
            int pixel_x = static_cast<int>((pt.x - local_x_min) / gray_map_resolution_);
            int pixel_y = static_cast<int>((pt.y - local_y_min) / gray_map_resolution_);
            pixel_x = std::max(0, std::min(width - 1, pixel_x));
            pixel_y = std::max(0, std::min(height - 1, pixel_y));
            gray_map.at<uint8_t>(height - 1 - pixel_y, pixel_x) = 0;
        }

        x_min = local_x_min;
        y_min = local_y_min;
        return !gray_map.empty();
    }

    void resetAccumulatedMap(const std::string& task_id)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        active_mapping_task_id_ = task_id;
        accumulated_map_points_.clear();
        accumulated_map_voxels_.clear();
    }

    void appendAccumulatedMap(const std::vector<SimplePoint>& cloud)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        for (const auto& point : cloud) {
            const VoxelKey key{
                static_cast<int>(std::floor(point.x / voxel_size_)),
                static_cast<int>(std::floor(point.y / voxel_size_)),
                static_cast<int>(std::floor(point.z / voxel_size_))
            };

            if (accumulated_map_voxels_.insert(key).second) {
                accumulated_map_points_.push_back(point);
            }
        }
    }

    bool exportAndUploadFullGrayMap(const std::string& task_id)
    {
        std::vector<SimplePoint> points_snapshot;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            points_snapshot = accumulated_map_points_;
        }

        if (points_snapshot.empty()) {
            ROS_WARN("Skipping global_map upload because accumulated map is empty.");
            return false;
        }

        cv::Mat gray_map;
        double x_min = 0.0;
        double y_min = 0.0;
        if (!buildGrayMapImage(points_snapshot, gray_map, x_min, y_min)) {
            ROS_ERROR("Failed to build full grayscale map for upload.");
            return false;
        }

        std::ostringstream oss;
        oss << "/tmp/global_map_" << task_id << ".png";
        const std::string output_path = oss.str();
        if (!cv::imwrite(output_path, gray_map)) {
            ROS_ERROR("Failed to write full grayscale map to %s", output_path.c_str());
            return false;
        }

        ROS_INFO("Full grayscale map exported to %s (origin_x=%.3f, origin_y=%.3f)",
                 output_path.c_str(), x_min, y_min);
        return uploadMapFile(task_id, output_path, "global_map");
    }

    void scheduleSaveMapUpload(const std::string& task_id)
    {
        const std::string map_file_path = global_map_file_path_;
        const bool use_external_map_file = !map_file_path.empty();
        const int retry_count = std::max(1, save_map_upload_retry_count_);
        const double delay_sec = std::max(0.0, save_map_upload_delay_sec_);

        std::thread([this, task_id, map_file_path, use_external_map_file, retry_count, delay_sec]() {
            for (int attempt = 1; attempt <= retry_count; ++attempt) {
                if (delay_sec > 0.0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(
                        static_cast<int>(delay_sec * 1000.0)));
                }

                bool success = false;
                if (use_external_map_file) {
                    success = uploadMapFile(task_id, map_file_path, "global_map");
                } else {
                    success = exportAndUploadFullGrayMap(task_id);
                }

                if (success) {
                    ROS_INFO("save_map upload succeeded on attempt %d for taskId=%s",
                             attempt, task_id.c_str());
                    return;
                }

                ROS_WARN("save_map upload attempt %d/%d failed for taskId=%s",
                         attempt, retry_count, task_id.c_str());
            }

            sendAlert("SAVE_MAP_UPLOAD_FAILED", "error", "global_map upload failed after retries");
        }).detach();
    }

    void heartbeatTimerCallback(const ros::TimerEvent&)
    {
        sendHeartbeat();
    }

    void statusTimerCallback(const ros::TimerEvent&)
    {
        sendStatus();
    }

    void incomingTextCallback(const std_msgs::String::ConstPtr& msg)
    {
        handleServerMessage(msg->data);
    }

    void alertTextCallback(const std_msgs::String::ConstPtr& msg)
    {
        std::string code = "EDGE_ALERT";
        std::string level = "warning";
        std::string message = msg->data;

        try {
            const json j = json::parse(msg->data);
            code = j.value("alertCode", code);
            level = j.value("level", level);
            message = j.value("message", message);
        } catch (...) {
            // 允许直接传纯文本，作为 message 使用。
        }

        sendAlert(code, level, message);
    }

    void handleServerMessage(const std::string& text)
    {
        try {
            const json j = json::parse(text);
            const std::string event_name = j.value("event", "");

            if (event_name == "register.ack") {
                handleRegisterAck(j);
                return;
            }

            if (event_name == "command") {
                handleCommand(j);
                return;
            }

            ROS_WARN("Received unhandled server event: %s", event_name.c_str());
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to parse server JSON: %s", e.what());
        }
    }

    void handleRegisterAck(const json& msg)
    {
        const bool success = msg.value("success", false);
        const std::string ack_robot_code = msg.value("robotCode", "");

        if (!success) {
            registered_ = false;
            ROS_ERROR("register.ack reported failure, robotCode=%s", ack_robot_code.c_str());
            return;
        }

        registered_ = true;

        std::string current_task_id_snapshot;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (msg.contains("currentTaskId") && msg["currentTaskId"].is_string()) {
                current_task_id_ = msg["currentTaskId"].get<std::string>();
            } else {
                current_task_id_.clear();
            }
            current_task_id_snapshot = current_task_id_;
        }

        ROS_INFO("Registration succeeded, current taskId=%s",
                 current_task_id_snapshot.empty() ? "<null>" : current_task_id_snapshot.c_str());
    }

    void handleCommand(const json& msg)
    {
        const std::string command_id = msg.value("commandId", "");
        const std::string task_id = msg.value("taskId", "");
        const std::string command = msg.value("command", "");
        const json payload = msg.value("payload", json::object());

        if (!task_id.empty()) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_task_id_ = task_id;
        }

        const bool accepted = processCommand(command, payload, task_id);
        if (auto_ack_supported_commands_ && !command_id.empty()) {
            sendCommandAck(command_id, task_id, accepted ? "accepted" : "failed");//reject？
        }
    }

    bool processCommand(const std::string& command,
                        const json& payload,
                        const std::string& task_id)
    {
        if (command == "start_precheck") {
            mode_ = "precheck";
            mapping_active_ = false;
            ROS_INFO("Received command start_precheck, entering precheck flow.");
            return true;
        }

        if (command == "start_mapping") {
            if (active_mapping_task_id_ != task_id) {
                resetAccumulatedMap(task_id);
            }
            mode_ = "mapping";
            mapping_active_ = true;
            ROS_INFO("Received command start_mapping, entering mapping flow.");
            return true;
        }

        if (command == "pause") {
            mode_ = "paused";
            mapping_active_ = false;
            ROS_INFO("Received command pause, mapping paused and current taskId is preserved.");
            return true;
        }

        if (command == "resume") {
            mode_ = "mapping";
            mapping_active_ = true;
            ROS_INFO("Received command resume, mapping resumed with the preserved taskId.");
            return true;
        }

        if (command == "save_map") {
            mode_ = "saving";
            mapping_active_ = false;
            ROS_INFO("Received command save_map, entering map saving flow.");

            if (task_id.empty()) {
                ROS_ERROR("save_map failed: taskId is empty.");
                sendAlert("SAVE_MAP_UPLOAD_FAILED", "error", "taskId is empty when uploading global_map");
                return false;
            }

            if (auto_upload_map_on_save_) {
                scheduleSaveMapUpload(task_id);
            }
            return true;
        }

        if (command == "cancel") {
            mode_ = "idle";
            mapping_active_ = false;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                current_task_id_.clear();
            }
            resetAccumulatedMap("");
            ROS_INFO("Received command cancel, current task context cleared.");
            return true;
        }

        if (command == "start_navigation") {
            if (!payload.contains("waypoints") || !payload["waypoints"].is_array() || payload["waypoints"].empty()) {
                ROS_ERROR("start_navigation failed: payload.waypoints is missing or empty.");
                return false;
            }

            int published_count = 0;
            for (const auto& waypoint : payload["waypoints"]) {
                if (!waypoint.is_object() ||
                    !waypoint.contains("x") || !waypoint["x"].is_number() ||
                    !waypoint.contains("y") || !waypoint["y"].is_number()) {
                    ROS_WARN("Skipping invalid waypoint in start_navigation payload.");
                    continue;
                }

                geometry_msgs::PoseStamped goal;
                goal.header.stamp = ros::Time::now();
                goal.header.frame_id = navigation_frame_id_;
                goal.pose.position.x = waypoint["x"].get<double>();
                goal.pose.position.y = waypoint["y"].get<double>();
                goal.pose.position.z = (waypoint.contains("z") && waypoint["z"].is_number())
                    ? waypoint["z"].get<double>()
                    : 0.0;

                double yaw_radians = 0.0;
                parseYawRadians(waypoint, yaw_radians);
                fillYawOrientation(yaw_radians, goal.pose.orientation);

                navigation_goal_pub_.publish(goal);
                ++published_count;
            }

            if (published_count == 0) {
                ROS_ERROR("start_navigation failed: no valid waypoints were published.");
                return false;
            }

            mode_ = "navigation";
            mapping_active_ = false;
            ROS_INFO("Received command start_navigation, published %d waypoint goal(s).", published_count);
            return true;
        }

        if (command == "stop_navigation") {
            actionlib_msgs::GoalID cancel_msg;
            cancel_msg.stamp = ros::Time::now();
            navigation_cancel_pub_.publish(cancel_msg);
            mode_ = "idle";
            ROS_INFO("Received command stop_navigation, published cancel to /move_base/cancel.");
            return true;
        }

        ROS_WARN("Received unknown command: %s", command.c_str());
        return false;
    }

    std::string buildUploadUrl(const std::string& task_id) const
    {
        const std::string placeholder = "{taskId}";
        std::string url = upload_url_template_;
        const std::string::size_type pos = url.find(placeholder);
        if (pos != std::string::npos) {
            url.replace(pos, placeholder.size(), task_id);
        }
        return url;
    }

    bool uploadMapFile(const std::string& task_id,
                       const std::string& file_path,
                       const std::string& file_role)
    {
        if (!fileExists(file_path)) {
            ROS_ERROR("Map file does not exist: %s", file_path.c_str());
            return false;
        }

        CURL* curl = curl_easy_init();
        if (curl == nullptr) {
            ROS_ERROR("Failed to initialize curl.");
            return false;
        }

        std::string response;
        const std::string url = buildUploadUrl(task_id);

        curl_mime* mime = curl_mime_init(curl);
        curl_mimepart* part = curl_mime_addpart(mime);
        curl_mime_name(part, "file");
        curl_mime_filedata(part, file_path.c_str());

        part = curl_mime_addpart(mime);
        curl_mime_name(part, "file_role");
        curl_mime_data(part, file_role.c_str(), CURL_ZERO_TERMINATED);

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlWriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 120L);

        const CURLcode res = curl_easy_perform(curl);
        long http_code = 0;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

        curl_mime_free(mime);
        curl_easy_cleanup(curl);

        if (res != CURLE_OK) {
            ROS_ERROR("Map upload failed, curl error: %s", curl_easy_strerror(res));
            return false;
        }

        if (http_code < 200 || http_code >= 300) {
            ROS_ERROR("Map upload failed, HTTP status=%ld, response=%s", http_code, response.c_str());
            return false;
        }

        ROS_INFO("Map upload succeeded, HTTP status=%ld, response=%s", http_code, response.c_str());
        return true;
    }

    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        quaternionToEuler(msg->pose.pose.orientation.x,
                          msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z,
                          msg->pose.pose.orientation.w,
                          roll, pitch, yaw);

        latest_roll_ = finiteOrDefault(roll, 0.0);
        latest_pitch_ = finiteOrDefault(pitch, 0.0);
        latest_yaw_ = finiteOrDefault(yaw, 0.0);
        latest_omega_z_ = finiteOrDefault(msg->twist.twist.angular.z, 0.0);
        latest_linear_x_ = finiteOrDefault(msg->twist.twist.linear.x, 0.0);
        latest_linear_y_ = finiteOrDefault(msg->twist.twist.linear.y, 0.0);
        latest_height_ = finiteOrDefault(msg->pose.pose.position.z, default_height_);

        if (!canSendTelemetry()) {
            return;
        }

        if (!mapping_active_) {
            return;
        }

        json j = buildBaseEvent("pose");
        j["data"]["position"]["x"] = msg->pose.pose.position.x;
        j["data"]["position"]["y"] = msg->pose.pose.position.y;
        j["data"]["position"]["z"] = msg->pose.pose.position.z;
        j["data"]["quaternion"]["x"] = msg->pose.pose.orientation.x;
        j["data"]["quaternion"]["y"] = msg->pose.pose.orientation.y;
        j["data"]["quaternion"]["z"] = msg->pose.pose.orientation.z;
        j["data"]["quaternion"]["w"] = msg->pose.pose.orientation.w;
        j["data"]["panorama"] = "";
        sendText(j);
    }

    bool encodeRosImageToJpegBase64(const sensor_msgs::Image::ConstPtr& msg,
                                    int& width,
                                    int& height,
                                    std::string& encoded_payload) const
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        cv::Mat bgr_image;
        if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
            bgr_image = cv_ptr->image;
        } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
            cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_RGB2BGR);
        } else if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
            cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_GRAY2BGR);
        } else {
            ROS_WARN_THROTTLE(5.0, "Unsupported image encoding for JPEG encode: %s", msg->encoding.c_str());
            return false;
        }

        if (bgr_image.empty()) {
            ROS_WARN_THROTTLE(5.0, "Skipping JPEG encode because input image is empty.");
            return false;
        }

        std::vector<uint8_t> jpeg_buffer;
        std::vector<int> params;
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(jpeg_quality_);
        if (!cv::imencode(".jpg", bgr_image, jpeg_buffer, params) || jpeg_buffer.empty()) {
            ROS_WARN("JPEG encoding failed.");
            return false;
        }

        width = bgr_image.cols;
        height = bgr_image.rows;
        encoded_payload = base64Encode(jpeg_buffer);
        return !encoded_payload.empty();
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        if (!canSendMappingTelemetry()) {
            return;
        }

        try {
            if (msg->point_step == 0 || msg->data.empty()) {
                ROS_WARN_THROTTLE(5.0, "Skipping /cloud_registered because the message is empty.");
                return;
            }

            int x_offset = -1;
            int y_offset = -1;
            int z_offset = -1;
            int rgb_offset = -1;
            int rgb_datatype = -1;
            for (const auto& field : msg->fields) {
                if (field.name == "x") {
                    x_offset = static_cast<int>(field.offset);
                } else if (field.name == "y") {
                    y_offset = static_cast<int>(field.offset);
                } else if (field.name == "z") {
                    z_offset = static_cast<int>(field.offset);
                } else if (field.name == "rgb" || field.name == "rgba") {
                    rgb_offset = static_cast<int>(field.offset);
                    rgb_datatype = static_cast<int>(field.datatype);
                }
            }

            if (x_offset < 0 || y_offset < 0 || z_offset < 0 ||
                x_offset + 4 > static_cast<int>(msg->point_step) ||
                y_offset + 4 > static_cast<int>(msg->point_step) ||
                z_offset + 4 > static_cast<int>(msg->point_step)) {
                ROS_WARN_THROTTLE(5.0, "Skipping /cloud_registered because x/y/z fields are invalid.");
                return;
            }

            const size_t point_count_from_data = msg->data.size() / msg->point_step;
            if (point_count_from_data == 0) {
                ROS_WARN_THROTTLE(5.0, "Skipping /cloud_registered because point_count_from_data is zero.");
                return;
            }

            std::vector<SimplePoint> cloud_raw;
            cloud_raw.reserve(point_count_from_data);

            for (size_t i = 0; i < point_count_from_data; ++i) {
                const uint8_t* point_ptr = msg->data.data() + i * msg->point_step;

                float x = 0.0f;
                float y = 0.0f;
                float z = 0.0f;
                std::memcpy(&x, point_ptr + x_offset, sizeof(float));
                std::memcpy(&y, point_ptr + y_offset, sizeof(float));
                std::memcpy(&z, point_ptr + z_offset, sizeof(float));

                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                    continue;
                }

                uint8_t r = 255;
                uint8_t g = 255;
                uint8_t b = 255;
                if (rgb_offset >= 0 && rgb_offset + 4 <= static_cast<int>(msg->point_step)) {
                    uint32_t rgb_packed = 0;
                    if (rgb_datatype == sensor_msgs::PointField::FLOAT32) {
                        float rgb_float = 0.0f;
                        std::memcpy(&rgb_float, point_ptr + rgb_offset, sizeof(float));
                        std::memcpy(&rgb_packed, &rgb_float, sizeof(uint32_t));
                    } else {
                        std::memcpy(&rgb_packed, point_ptr + rgb_offset, sizeof(uint32_t));
                    }
                    r = static_cast<uint8_t>((rgb_packed >> 16) & 0xFF);
                    g = static_cast<uint8_t>((rgb_packed >> 8) & 0xFF);
                    b = static_cast<uint8_t>(rgb_packed & 0xFF);
                }

                cloud_raw.push_back(SimplePoint{x, y, z, r, g, b});
            }

            if (cloud_raw.empty()) {
                ROS_WARN_THROTTLE(5.0, "Skipping /cloud_registered because all parsed points are invalid.");
                return;
            }

            std::vector<SimplePoint> cloud_downsampled;
            cloud_downsampled.reserve(cloud_raw.size());

            if (voxel_size_ <= 0.0) {
                cloud_downsampled = cloud_raw;
            } else {
                std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels;
                occupied_voxels.reserve(cloud_raw.size());

                for (const auto& point : cloud_raw) {
                    const VoxelKey key{
                        static_cast<int>(std::floor(point.x / voxel_size_)),
                        static_cast<int>(std::floor(point.y / voxel_size_)),
                        static_cast<int>(std::floor(point.z / voxel_size_))
                    };

                    if (occupied_voxels.insert(key).second) {
                        cloud_downsampled.push_back(point);
                    }
                }
            }

            if (cloud_downsampled.empty()) {
                return;
            }

            appendAccumulatedMap(cloud_downsampled);

            if (enable_local_cloud_) {
                const size_t point_stride = 16;
                std::vector<uint8_t> raw_payload(cloud_downsampled.size() * point_stride, 0);
                for (size_t i = 0; i < cloud_downsampled.size(); ++i) {
                    const auto& pt = cloud_downsampled[i];
                    uint8_t* ptr = raw_payload.data() + i * point_stride;
                    std::memcpy(ptr + 0, &pt.x, sizeof(float));
                    std::memcpy(ptr + 4, &pt.y, sizeof(float));
                    std::memcpy(ptr + 8, &pt.z, sizeof(float));
                    ptr[12] = pt.r;
                    ptr[13] = pt.g;
                    ptr[14] = pt.b;
                    ptr[15] = 0;
                }

                const int src_size = static_cast<int>(raw_payload.size());
                const int max_compressed_size = LZ4_compressBound(src_size);
                if (max_compressed_size <= 0) {
                    ROS_ERROR("Failed to compute max LZ4 compressed size.");
                    return;
                }

                std::vector<char> compressed_payload(max_compressed_size);
                const int compressed_size = LZ4_compress_default(
                    reinterpret_cast<const char*>(raw_payload.data()),
                    compressed_payload.data(),
                    src_size,
                    max_compressed_size);

                if (compressed_size <= 0) {
                    ROS_ERROR("Point cloud LZ4 compression failed.");
                    return;
                }
                compressed_payload.resize(compressed_size);

                json j = buildBaseEvent("localCloud");
                j["data"] = {
                    {"source", "omni_livo_incremental_cloud"},
                    {"frameId", msg->header.frame_id},
                    {"pointCount", cloud_downsampled.size()},
                    {"pointFormat", "xyz_rgb_f32_u8"},
                    {"compression", "lz4"},
                    {"payload", base64Encode(compressed_payload)}
                };
                sendText(j);
            }

            // 降采样后的增量点云再投影成二维灰度子图，便于前端快速查看局部建图结果。
            sendGlobalSubmapGrayImage(cloud_downsampled, msg->header);
        } catch (const std::exception& e) {
            ROS_ERROR("pointCloudCallback exception: %s", e.what());
        }
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        if (!canSendMappingTelemetry()) {
            return;
        }

        if (!enable_video_frame_) {
            return;
        }

        try {
            int width = 0;
            int height = 0;
            std::string payload;
            if (!encodeRosImageToJpegBase64(msg, width, height, payload)) {
                return;
            }

            json j = buildBaseEvent("video.frame");
            j["data"] = {
                {"cameraId", camera_id_},
                {"frameNo", static_cast<uint64_t>(msg->header.seq)},
                {"width", width},
                {"height", height},
                {"format", "jpeg"},
                {"quality", jpeg_quality_},
                {"payload", payload}
            };
            sendText(j);
        } catch (const cv::Exception& e) {
            ROS_ERROR("OpenCV image encode/conversion failed: %s", e.what());
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge image conversion failed: %s", e.what());
        } catch (const std::exception& e) {
            ROS_ERROR("imageCallback exception: %s", e.what());
        }
    }

    void stitchCaptureCallback(const omni_stitch_capture::StitchedCapture::ConstPtr& msg)
    {
        if (!msg) {
            return;
        }

        if (!canSendTelemetry()) {
            return;
        }

        try {
            sensor_msgs::Image::ConstPtr image_msg(new sensor_msgs::Image(msg->image));
            int width = 0;
            int height = 0;
            std::string panorama_payload;
            if (!encodeRosImageToJpegBase64(image_msg, width, height, panorama_payload)) {
                return;
            }

            (void)width;
            (void)height;

            json j = buildBaseEvent("pose");
            j["data"]["position"]["x"] = msg->pose.position.x;
            j["data"]["position"]["y"] = msg->pose.position.y;
            j["data"]["position"]["z"] = msg->pose.position.z;
            j["data"]["quaternion"]["x"] = msg->pose.orientation.x;
            j["data"]["quaternion"]["y"] = msg->pose.orientation.y;
            j["data"]["quaternion"]["z"] = msg->pose.orientation.z;
            j["data"]["quaternion"]["w"] = msg->pose.orientation.w;
            j["data"]["panorama"] = panorama_payload;
            sendText(j);
        } catch (const cv::Exception& e) {
            ROS_ERROR("OpenCV stitch capture encode/conversion failed: %s", e.what());
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge stitch capture conversion failed: %s", e.what());
        } catch (const std::exception& e) {
            ROS_ERROR("stitchCaptureCallback exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "edge_agent_node");
    ros::NodeHandle nh("~");

    EdgeAgent agent(nh);
    ros::spin();
    return 0;
}
