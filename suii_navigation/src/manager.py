#! /usr/bin/env python

import rospy
import time
from math import cos, sin
from std_srvs.srv import Empty
from suii_msgs.srv import NavigationGoal, NavigationRepose
from actionlib import GoalStatus, SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import TransformListener
from tf.transformations import euler_from_quaternion


class NavigationManager(object):
    def __init__(self): #Initializes serveces and clients
        rospy.init_node("NavManager_node", anonymous=True)
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server(rospy.Duration(5))

        rospy.Service('move_to_goal', NavigationGoal, self.goal_handler)
        rospy.Service('reposition', NavigationRepose, self.repose_handler)

        self.reset_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.tf_listener = TransformListener()

        self.goal = MoveBaseGoal()
	#rospy.Timer(rospy.Duration(secs=3,nsecs=0), self.clear_costmap)
        rospy.spin()


    def goal_handler(self, data): #Processes the NavigationGoal service
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = data.pose.position.x
        self.goal.target_pose.pose.position.y = data.pose.position.y
        self.goal.target_pose.pose.orientation.z = data.pose.orientation.z
        self.goal.target_pose.pose.orientation.w = data.pose.orientation.w

        status = self.send_goal()
        return status
        

    def repose_handler(self, data): #Processes the NavigationRepose service
        x_req = data.target.x
        y_req = data.target.y
        linear, angular = self.get_repose_goal_coords(x_req, y_req)

        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = linear[0]
        self.goal.target_pose.pose.position.y = linear[1]
        self.goal.target_pose.pose.orientation.z = angular[2]
        self.goal.target_pose.pose.orientation.w = angular[3]

        status = self.send_goal()
        return status


    def send_goal(self): #Sends a goal to move_base using actionlib
        self.clear_costmap()
        time.sleep(0.2)
        self.move_base_client.send_goal(self.goal)
        status = self.move_base_client.wait_for_result()

        if not status:
            self.move_base_client.cancel_goal()
            rospy.loginfo("move_base failed")
            return 1 #False
        else:
            state = self.move_base_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached")
                return 0 #True
            rospy.loginfo("Can't reach goal")
            return 1 #False 


    def get_repose_goal_coords(self, x_req, y_req): #Uses the robots actual postion to calcualte the new reposition goal
        (linear, angular) = self.tf_listener.lookupTransform('base_link', 'map', rospy.Time(0))
        current_x = linear[0]
        current_y = linear[1]
        current_theta = euler_from_quaternion(angular)

        new_x = ( (cos(current_theta[2]) * x_req) - (sin(current_theta[2]) * y_req) )
        new_y = ( (cos(current_theta[2]) * y_req) + (sin(current_theta[2]) * x_req) )
        linear[0] += new_x
        linear[1] += new_y
        return linear, angular


    def clear_costmap(self): #Calls the clear costmap service
        self.reset_costmap()
        rospy.loginfo('Costmap cleared')


if __name__ == "__main__": 
    NavigationManager()

