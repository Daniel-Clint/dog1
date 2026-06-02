#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class PlanExploreTrigger:
    def __init__(self):
        rospy.init_node('plan_explore_trigger_node', anonymous=True)
        
        self.map_msg = None
        self.current_mode = 'IDLE' 
        self.target_goal = None
        
        # Subscribe to map and simple goal
        self.map_sub = rospy.Subscriber('/projected_map', OccupancyGrid, self.map_cb)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        
        # Action client to move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server. Ready to process goals.")

    def map_cb(self, msg):
        self.map_msg = msg

    def check_if_free(self, world_x, world_y):
        if self.map_msg is None:
            rospy.logwarn("No map received yet. Treating point as unknown.")
            return False
            
        res = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        
        grid_x = int((world_x - origin_x) / res)
        grid_y = int((world_y - origin_y) / res)
        
        # Check if coordinates are out of the map bounds
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            return False
            
        index = grid_x + grid_y * width
        val = self.map_msg.data[index]
        
        # OccupancyGrid values: 0 is known free, 100 is occupied, -1 is unknown
        # Any value between 0 and 49 is generally considered free or very low probability of occupation.
        if val >= 0 and val < 50:
            return True
            
        return False

    def goal_cb(self, msg):
        world_x = msg.pose.position.x
        world_y = msg.pose.position.y
        
        # Check if goal is in a known free space
        if self.check_if_free(world_x, world_y):
            rospy.loginfo(f"Goal ({world_x:.2f}, {world_y:.2f}) is in known free space. Direct Navigation.")
            self.current_mode = 'NAVIGATE'
            self.target_goal = msg
        else:
            rospy.loginfo(f"Goal ({world_x:.2f}, {world_y:.2f}) is in unknown/occupied space. Starting Exploration Loop.")
            self.current_mode = 'EXPLORE'
            
        # Cancel any ongoing goals to interrupt current task (including move_base's own reaction to simple_goal)
        self.client.cancel_all_goals()

    def run(self):
        rate = rospy.Rate(5) 
        waited_for_abort = False

        while not rospy.is_shutdown():
            if self.current_mode == 'IDLE':
                pass

            elif self.current_mode == 'NAVIGATE':
                if self.target_goal is not None:
                    # Construct and send the action goal
                    mb_goal = MoveBaseGoal()
                    mb_goal.target_pose = self.target_goal
                    
                    # Prevent accidental trigger of exploration via our custom z-hack
                    if mb_goal.target_pose.pose.position.z >= 1000.0:
                        mb_goal.target_pose.pose.position.z = 0.0
                        
                    self.client.send_goal(mb_goal)
                    rospy.loginfo("Navigation target sent to move_base.")
                    self.target_goal = None
                
            elif self.current_mode == 'EXPLORE':
                state = self.client.get_state()
                
                # Check status
                if state in [actionlib.GoalStatus.PENDING, actionlib.GoalStatus.ACTIVE]:
                    # Currently executing the exploration trajectory
                    waited_for_abort = False
                else:
                    # Not actively exploring
                    if state not in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.LOST]:
                        if not waited_for_abort:
                            rospy.logwarn(f"Did not reach the frontier successfully (State: {state}). Retrying in 2 seconds...")
                            rospy.sleep(2.0)
                            waited_for_abort = True
                            continue
                            
                    waited_for_abort = False
                    
                    # Prevent race condition if mode was changed during sleep
                    if self.current_mode != 'EXPLORE':
                        continue
                        
                    # Issue the next dummy goal to trigger the exploration logic
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = 0.0
                    goal.target_pose.pose.position.y = 0.0
                    goal.target_pose.pose.position.z = 1337.0 # Triggers exploration internal hook
                    goal.target_pose.pose.orientation.w = 1.0
                    goal.target_pose.pose.orientation.x = 0.0
                    goal.target_pose.pose.orientation.y = 0.0
                    goal.target_pose.pose.orientation.z = 0.0

                    rospy.loginfo("Sending dummy goal to trigger autonomous exploration...")
                    self.client.send_goal(goal)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = PlanExploreTrigger()
        node.run()
    except rospy.ROSInterruptException:
        pass
