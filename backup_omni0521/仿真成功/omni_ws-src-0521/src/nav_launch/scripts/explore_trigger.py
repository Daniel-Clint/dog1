#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

"""
1. move_base 的目标管理逻辑
    move_base 是一个典型的“目标驱动型”状态机。

    当你发送一个 Goal(无论是真实的坐标还是我们的 Dummy Goal), 
    move_base 会进入 ACTIVE 状态, 并调用全局规划器(Hector)寻找路径。
    一旦机器人到达了全局规划器选定的那个“边界点”(Frontier), 
    move_base 就会认为当前任务已经完成, 状态变为 SUCCEEDED, 然后停止所有驱动输出。
    如果不发送下一个 Goal, move_base 就会一直停在那里。
2. 探索是一个“动态发现”的过程
    自主探索本质上是由一系列“发现 -> 移动 -> 再次发现”组成的: 

    第一次触发: 发一个 Dummy Goal。Hector 插件在当前地图中找到离你最近的一个边界 A。
    执行中: 机器人开始向 A 点移动, 在这个过程中, 传感器会不断扫描, 地图会不断更新。
    到达 A 点: 此时 move_base 任务结束。但地图上还有更多未知的边界(B, C, D...)。
    循环触发: explore_trigger.py 检测到上一个任务结束, 立刻再发一个 Dummy Goal。
    此时 Hector 会基于最新的、更大的地图重新计算, 找到下一个最优边界 B。
"""
def main():
    rospy.init_node('explore_trigger_node', anonymous=True)
    
    # Create action client for move_base
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base action server. Starting exploration loop.")

    rate = rospy.Rate(1) # Check rate
    
    while not rospy.is_shutdown():
        # Create a dummy goal with zero quaternion to trigger hector_exploration_planner's doExploration()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = 0.0
        goal.target_pose.pose.position.y = 0.0
        goal.target_pose.pose.position.z = 1337.0 # Triggers exploration in our custom plugin
        goal.target_pose.pose.orientation.w = 1.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0

        rospy.loginfo("Sending dummy goal to trigger exploration...")
        client.send_goal(goal)
        
        # Wait for the robot to reach the generated frontier
        client.wait_for_result()
        
        state = client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached the frontier.")
        else:
            rospy.logwarn("Did not reach the frontier successfully (State: {}). Retrying after 2 seconds.".format(state))
            rospy.sleep(2.0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
