#!/usr/bin/env python3
import rospy
import struct
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class GlobalMapSaver:
    def __init__(self):
        rospy.init_node('pcd_saver_node', anonymous=True)
        
        # 订阅 Omni-LIVO 发布对齐后点云的话题
        self.topic = rospy.get_param('~topic', '/cloud_registered')
        self.save_path = rospy.get_param('~save_path', 'omni_livo_global_map.pcd')
        
        self.all_points = []
        self.has_rgb = False
        self.has_intensity = False
        
        rospy.Subscriber(self.topic, PointCloud2, self.pc_callback)
        rospy.loginfo(f"✅ 成功启动！正在监听话题: {self.topic}")
        rospy.loginfo("⚠️ 请让 SLAM 算法继续跑，等你想保存时，在此终端按 Ctrl+C 即可！")
        
        # 注册退出时的回调函数，用来保存 PCD
        rospy.on_shutdown(self.save_pcd)

    def pc_callback(self, msg):
        field_names = [f.name for f in msg.fields]
        
        if 'rgb' in field_names:
            self.has_rgb = True
            gen = pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
            for p in gen:
                # 解析 ROS 里被打包成 float 的 RGB 数据
                packed_rgb = struct.pack('f', p[3])
                # 注意：ROS 的 RGB float 在内存中通常是 B, G, R, A 顺序（小端序）
                b, g, r, _ = struct.unpack('BBBB', packed_rgb)
                
                # 直接通过位移运算，拼接成 PCL 认识的 U32 整数 (0x00RRGGBB)
                # 这样可以 100% 无损保留真实的颜色色彩，拒绝科学计数法四舍五入！
                rgb_int = (r << 16) | (g << 8) | b
                self.all_points.append((p[0], p[1], p[2], rgb_int))
                
        elif 'intensity' in field_names:
            self.has_intensity = True
            gen = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
            for p in gen:
                self.all_points.append((p[0], p[1], p[2], p[3]))
                
        else:
            gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            for p in gen:
                self.all_points.append((p[0], p[1], p[2]))
                
        # 避免刷屏的日志打印逻辑
        if len(self.all_points) % 50000 < 2000:
            rospy.loginfo(f"⏳ 正在接收数据，当前已收集 {len(self.all_points)} 个点...")

    def save_pcd(self):
        if not self.all_points:
            rospy.logwarn("❌ 没有收到任何点云数据，保存失败！请检查话题名称是否正确。")
            return
            
        rospy.loginfo(f"\n💾 收到退出指令！正在将 {len(self.all_points)} 个点保存到 {self.save_path} ...")
        
        # 写入 PCD ASCII 格式文件
        with open(self.save_path, 'w') as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            
            if self.has_rgb:
                f.write("FIELDS x y z rgb\n")
                f.write("SIZE 4 4 4 4\n")
                # 【关键修复】: 这里的类型改成 F F F U (U 代表 Unsigned int)
                f.write("TYPE F F F U\n")
                f.write("COUNT 1 1 1 1\n")
            elif self.has_intensity:
                f.write("FIELDS x y z intensity\n")
                f.write("SIZE 4 4 4 4\n")
                f.write("TYPE F F F F\n")
                f.write("COUNT 1 1 1 1\n")
            else:
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
                
            f.write(f"WIDTH {len(self.all_points)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(self.all_points)}\n")
            f.write("DATA ascii\n")
            
            for p in self.all_points:
                if self.has_rgb:
                    # 【关键修复】: 直接输出整数 p[3]，彻底杜绝浮点精度丢失！
                    f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[3]}\n")
                elif self.has_intensity:
                    f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[3]:.6f}\n")
                else:
                    f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
                    
        rospy.loginfo("✅ 保存完成！使用 pcl_viewer 查看时记得按 '+' 号放大点云以获得最佳色彩效果。")

if __name__ == '__main__':
    saver = GlobalMapSaver()
    rospy.spin()