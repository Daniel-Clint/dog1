#!/usr/bin/env python
import rospy
import socket
import struct
import numpy as np
import cv2
from sensor_msgs2.msg import CompressedImage

class TCPImageClient:
    def __init__(self, server_ip, server_port):
        self.server_ip = server_ip
        self.server_port = server_port
        self.sock = None
        self.connect()
        cv2.namedWindow('Received Image', cv2.WINDOW_NORMAL)

    def connect(self):
        """连接到服务器"""
        if self.sock:
            self.sock.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((self.server_ip, self.server_port))
            print(f"Connected to {self.server_ip}:{self.server_port}")
            # 禁用Nagle算法
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except socket.error as e:
            print(f"Connection failed: {e}")
            self.sock = None

    def receive_images(self):
        while True:
            if not self.sock:
                sleep(1)
                self.connect()
                continue

            try:
                # 读取长度头（4字节）
                header = self.sock.recv(4)
                if len(header) != 4:
                    if len(header) == 0:
                        print("Connection closed by server")
                        self.sock = None
                        continue
                    print("Invalid header size")
                    continue

                # 解析数据长度
                data_size = struct.unpack('!I', header)[0]
                
                # 读取完整数据
                data = bytearray()
                while len(data) < data_size:
                    remaining = data_size - len(data)
                    chunk = self.sock.recv(4096 if remaining > 4096 else remaining)
                    if not chunk:
                        print("Connection closed during data transfer")
                        self.sock = None
                        break
                    data.extend(chunk)
                
                if len(data) != data_size:
                    print(f"Incomplete data: expected {data_size}, got {len(data)}")
                    continue
                
                # 反序列化ROS消息
                compressed_img = CompressedImage()
                compressed_img.deserialize(data)
                
                # 转换为OpenCV图像
                np_arr = np.frombuffer(compressed_img.data, np.uint8)
                cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if cv_img is not None:
                    cv2.imshow("Received Image", cv_img)
                    cv2.waitKey(1)
                else:
                    rospy.logerr("Failed to decode image")
                    
            except socket.error as e:
                rospy.logerr(f"Socket error: {e}")
                self.sock = None
            except Exception as e:
                rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    client = TCPImageClient("192.168.1.2", 8888)  # 替换为C++服务器IP
    client.receive_images()
