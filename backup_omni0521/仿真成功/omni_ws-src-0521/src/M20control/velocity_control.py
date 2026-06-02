#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import socket
import json
import time
from geometry_msgs.msg import Twist


class CmdVelToUDP:
    def __init__(self):
        # 下位机 IP 和端口
        self.server_ip = rospy.get_param("~server_ip", "10.21.31.103")
        self.port = rospy.get_param("~port", 30000)

        # cmd_vel 话题名
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        # 安全限幅
        self.max_x = rospy.get_param("~max_x", 0.5)
        self.max_y = rospy.get_param("~max_y", 0.5)
        self.max_yaw = rospy.get_param("~max_yaw", 0.8)

        # 超时保护：多久没收到 cmd_vel 就发 0
        self.timeout = rospy.get_param("~timeout", 0.3)

        self.last_cmd_time = rospy.Time.now()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        rospy.Subscriber(
            self.cmd_vel_topic,
            Twist,
            self.cmd_vel_callback,
            queue_size=1
        )

        # 20Hz 定时检查是否超时
        self.timer = rospy.Timer(
            rospy.Duration(0.05),
            self.watchdog_callback
        )

        rospy.loginfo("cmd_vel_to_udp node started")
        rospy.loginfo("Subscribing topic: %s", self.cmd_vel_topic)
        rospy.loginfo("UDP target: %s:%d", self.server_ip, self.port)

    def build_header(self, data_len):
        h = bytearray(16)

        # 固定帧头
        h[0], h[1], h[2], h[3] = 0xeb, 0x91, 0xeb, 0x90

        # 数据长度，小端
        h[4] = data_len & 0xFF
        h[5] = (data_len >> 8) & 0xFF

        # 序号/版本之类，参考你原来的协议
        h[6], h[7] = 1, 0

        # 固定字段
        h[8] = 0x01

        return h

    def limit(self, value, max_abs):
        if value > max_abs:
            return max_abs
        if value < -max_abs:
            return -max_abs
        return value

    def send_velocity(self, x=0.0, y=0.0, yaw=0.0):
        """
        按照下位机协议发送速度指令
        """

        x = self.limit(x, self.max_x)
        y = self.limit(y, self.max_y)
        yaw = self.limit(yaw, self.max_yaw)

        t_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

        payload = {
            "PatrolDevice": {
                "Type": 2,
                "Command": 21,
                "Time": t_str,
                "Items": {
                    "X": float(x),
                    "Y": float(y),
                    "Z": 0.0,
                    "Roll": 0.0,
                    "Pitch": 0.0,
                    "Yaw": float(yaw)
                }
            }
        }

        j_str = json.dumps(
            payload,
            separators=(',', ':')
        ).encode("utf-8")

        pkt = self.build_header(len(j_str)) + j_str

        self.sock.sendto(pkt, (self.server_ip, self.port))

    def cmd_vel_callback(self, msg):
        """
        订阅 /cmd_vel 后，将 Twist 转换为下位机协议
        """

        x = msg.linear.x
        y = msg.linear.y
        yaw = msg.angular.z

        self.last_cmd_time = rospy.Time.now()

        self.send_velocity(x=x, y=y, yaw=yaw)

        rospy.logdebug(
            "Send cmd_vel -> UDP: X=%.3f, Y=%.3f, Yaw=%.3f",
            x, y, yaw
        )

    def watchdog_callback(self, event):
        """
        超时保护：
        如果超过 timeout 没收到 /cmd_vel，就持续发 0 速度
        """

        now = rospy.Time.now()
        dt = (now - self.last_cmd_time).to_sec()

        if dt > self.timeout:
            self.send_velocity(0.0, 0.0, 0.0)

    def shutdown(self):
        rospy.loginfo("Shutting down, sending stop command...")

        for _ in range(10):
            self.send_velocity(0.0, 0.0, 0.0)
            time.sleep(0.05)

        self.sock.close()


def main():
    rospy.init_node("cmd_vel_to_udp")

    node = CmdVelToUDP()

    rospy.on_shutdown(node.shutdown)

    rospy.spin()


if __name__ == "__main__":
    main()
