#! /usr/bin/env python

import rospy
from math import sqrt
from tf import TransformListener
from suii_msgs.srv import DistanceToGoal
from tf import TransformListener
from tf.transformations import euler_from_quaternion
from suii_control.utils.file_handler import FileHandler 

class EuclideanDistanceCalculator(object):
    def __init__(self):
        rospy.init_node("euclidean_calculator_node", anonymous=True)
        rospy.Service('get_distance', DistanceToGoal, self.get_distance_handler)
        self.tf_listener = TransformListener()
        
        self.locations = None
        self.yaml_read()

        rospy.spin()
        

    def get_distance_handler(self, waypoint_data):
        position = self.get_coords_from_file(waypoint_data)
        distance = self.calculate_euclidean_distance(position)
        return distance


    def get_coords_from_file(self, waypoint_data):
        for item in self.locations:
            if ((item.destination.area_type == waypoint_data.type) and (item.destination.instance_id == waypoint_data.instance_id)):
                return item.destination.position
            else:
                print('No coordinates found!')
            

    def calculate_euclidean_distance(self, position):
        (linear, angular) = self.tf_listener.lookupTransform('base_link', 'map', rospy.time(0))
        current_x = linear[0]
        current_y = linear[1]

        distance = abs(sqrt( ( (position.x - current_x) ** 2) + ( (position.y - current_y) ** 2) ) )
        return distance


    def yaml_read(self):
        file_handler = FileHandler()
        if rospy.has_param('/suii_control_node/path'):
            file_name = rospy.get_param('/suii_control_node/path')
            self.locations = file_handler.load(file_name)
            # print yaml.dump(self.navigation_tasks_config)
        else:
            print('Could not find parameter!')


if __name__ == "__main__": 
    EuclideanDistanceCalculator()

