#!/usr/bin/env python
import rospy
import socket
import struct
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image

class TCPImageClient:
    def __init__(self, server_ip, server_port):
        self.server_ip = server_ip
        self.server_port = server_port
        self.sock = None
        self.connect()
        self.bridge = CvBridge()

        # 创建常规图像发布器
        self.top_half_pub = rospy.Publisher('/fisheye/left/image_raw', Image, queue_size=1)
        self.bottom_half_pub = rospy.Publisher('/fisheye/right/image_raw', Image, queue_size=1)
        
        # 创建压缩图像发布器
        self.top_half_compressed_pub = rospy.Publisher('/fisheye/left/image_raw/compressed', CompressedImage, queue_size=1)
        self.bottom_half_compressed_pub = rospy.Publisher('/fisheye/right/image_raw/compressed', CompressedImage, queue_size=1)
        

    def connect(self):
        """连接到服务器"""
        if self.sock:
            self.sock.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((self.server_ip, self.server_port))
            rospy.loginfo(f"Connected to {self.server_ip}:{self.server_port}")
            # 禁用Nagle算法
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except socket.error as e:
            rospy.logerr(f"Connection failed: {e}")
            self.sock = None

    def receive_images(self):
        while not rospy.is_shutdown():
            if not self.sock:
                rospy.sleep(1)
                self.connect()
                continue

            try:
                # 读取长度头（4字节）
                header = self.sock.recv(4)
                if len(header) != 4:
                    if len(header) == 0:
                        rospy.logwarn("Connection closed by server")
                        self.sock = None
                        continue
                    rospy.logwarn("Invalid header size")
                    continue

                # 解析数据长度
                data_size = struct.unpack('!I', header)[0]
                
                # 读取完整数据
                data = bytearray()
                while len(data) < data_size:
                    remaining = data_size - len(data)
                    chunk = self.sock.recv(4096 if remaining > 4096 else remaining)
                    if not chunk:
                        rospy.logwarn("Connection closed during data transfer")
                        self.sock = None
                        break
                    data.extend(chunk)
                
                if len(data) != data_size:
                    rospy.logwarn(f"Incomplete data: expected {data_size}, got {len(data)}")
                    continue
                
                # 反序列化ROS消息
                compressed_img = CompressedImage()
                compressed_img.deserialize(data)
                
                # 转换为OpenCV图像
                np_arr = np.frombuffer(compressed_img.data, np.uint8)
                cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if cv_img is not None:
                    height, width = cv_img.shape[:2]
                    split_point = height // 2
                    # 分割图像
                    top_half = cv_img[0:split_point, :]
                    bottom_half = cv_img[split_point:height, :]
                    # 发布压缩分割图像

                    self.publish_image(top_half, self.top_half_pub, "top_half", compressed_img.header)
                    self.publish_image(bottom_half, self.bottom_half_pub, "bottom_half", compressed_img.header)

                    self.publish_compressed(top_half, self.top_half_compressed_pub, "top_half", compressed_img.header)
                    self.publish_compressed(bottom_half, self.bottom_half_compressed_pub, "bottom_half", compressed_img.header)
                else:
                    rospy.logerr("Failed to decode image")
                    
            except socket.error as e:
                rospy.logerr(f"Socket error: {e}")
                self.sock = None
            except Exception as e:
                rospy.logerr(f"Error processing image: {e}")

    def publish_image(self, image, publisher, name, header):
        """发布原始图像消息"""
        try:
            msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            msg.header = header
            publisher.publish(msg)
        except Exception as e:
            rospy.logerr("Error publishing %s image: %s", name, str(e))
    
    def publish_compressed(self, image, publisher, name, header):
        """发布压缩图像消息"""
        try:
            msg = self.bridge.cv2_to_compressed_imgmsg(image, "jpeg")
            msg.header = header
            publisher.publish(msg)
        except Exception as e:
            rospy.logerr("Error publishing compressed %s image: %s", name, str(e))

if __name__ == '__main__':
    rospy.init_node('tcp_image_client')
    client = TCPImageClient("192.168.1.2", 8888)  # 替换为C++服务器IP
    client.receive_images()
