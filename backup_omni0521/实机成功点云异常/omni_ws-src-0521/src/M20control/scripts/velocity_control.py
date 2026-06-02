#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import socket
import time

import rospy
from geometry_msgs.msg import Twist


class CmdVelToUDP:
    def __init__(self):
        self.server_ip = rospy.get_param("~server_ip", "10.21.31.103")
        self.port = rospy.get_param("~port", 30000)
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.max_x = rospy.get_param("~max_x", 0.5)
        self.max_y = rospy.get_param("~max_y", 0.5)
        self.max_yaw = rospy.get_param("~max_yaw", 0.8)
        self.timeout = rospy.get_param("~timeout", 0.3)

        self.last_cmd_time = rospy.Time.now()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.watchdog_callback)

        rospy.loginfo("cmd_vel_to_udp node started")
        rospy.loginfo("Subscribing topic: %s", self.cmd_vel_topic)
        rospy.loginfo("UDP target: %s:%d", self.server_ip, self.port)

    @staticmethod
    def build_header(data_len):
        header = bytearray(16)
        header[0], header[1], header[2], header[3] = 0xEB, 0x91, 0xEB, 0x90
        header[4] = data_len & 0xFF
        header[5] = (data_len >> 8) & 0xFF
        header[6], header[7] = 1, 0
        header[8] = 0x01
        return header

    @staticmethod
    def limit(value, max_abs):
        if value > max_abs:
            return max_abs
        if value < -max_abs:
            return -max_abs
        return value

    def send_velocity(self, x=0.0, y=0.0, yaw=0.0):
        x = self.limit(x, self.max_x)
        y = self.limit(y, self.max_y)
        yaw = self.limit(yaw, self.max_yaw)

        payload = {
            "PatrolDevice": {
                "Type": 2,
                "Command": 21,
                "Time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
                "Items": {
                    "X": float(x),
                    "Y": float(y),
                    "Z": 0.0,
                    "Roll": 0.0,
                    "Pitch": 0.0,
                    "Yaw": float(yaw),
                },
            }
        }

        json_bytes = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        packet = self.build_header(len(json_bytes)) + json_bytes
        self.sock.sendto(packet, (self.server_ip, self.port))

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = rospy.Time.now()
        self.send_velocity(x=msg.linear.x, y=msg.linear.y, yaw=msg.angular.z)

    def watchdog_callback(self, _event):
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > self.timeout:
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
