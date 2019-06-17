#! /usr/bin/env python

import rospy
from math import sqrt
from tf import TransformListener
from suii_msgs.srv import DistanceToGoal
from tf import TransformListener
from tf.transformations import euler_from_quaternion

class EuclideanDistanceCalculator(object):
    def __init__(self):
        rospy.init_node("euclidean_calculator_node", anonymous=True)
        rospy.Service('get_distance', DistanceToGoal, self.get_distance_handler)
        self.tf_listener = TransformListener()
        
        self.locations = None
        rospy.spin()


    def get_distance_handler(self, waypoint_data):
        distance = self.calculate_euclidean_distance(waypoint_data.x, waypoint_data.y)
        return distance         


    def calculate_euclidean_distance(self, x_goal, y_goal):
        (linear, angular) = self.tf_listener.lookupTransform('base_link', 'map', rospy.time(0))
        current_x = linear[0]
        current_y = linear[1]

        distance = abs(sqrt( ( (x_goal - current_x) ** 2) + ( (y_goal - current_y) ** 2) ) )
        return distance


if __name__ == "__main__": 
    EuclideanDistanceCalculator()

