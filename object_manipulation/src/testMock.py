#!/usr/bin/env python

import rospy
import tf
import numpy as np

#services

#vision
from suii_msgs.srv import VisionScan, VisionScanRequest, VisionScanResponse
#ur3
from suii_msgs.srv import ManipulationAction, ManipulationActionRequest, ManipulationActionResponse
from suii_msgs.srv import ManipulationPose, ManipulationPoseRequest, ManipulationPoseResponse
from suii_msgs.msg import VisionScanObject

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

from suii_msgs.srv import NavigationGoal, NavigationRepose
from suii_msgs.srv import DistanceToGoal

_ids = [
    "F20_20_B",
    "F20_20_G",
    "S40_40_B",
    "S40_40_G",
    "M20_100",
    "M20",
    "M30",
    "R20",
    "Bearing_Box",
    "Bearing",
    "Axis",
    "Distance_Tube",
    "Motor",
    "F20_20_Mall",
    "F40_40_Mall",
    "R20_Mall",
    "Bolt_Mall",
    "M20_Mall",
    "M30_Mall",
    "Red_Container_Top",
    "Red_Container_Front",
    "Blue_Container_Top",
    "Blue_Container_Front"
]

_tfpos = [[0.5, 0.2, 0],
[0.2, 0.4, 0],
[0.4, 0.6, 0],
[0.7, 0.2, 0], 
[0.1, 0.9, 0],
[0.4, -0.5, 0],
[0.4, 0.6, 0],
[0.7, 0.2, 0], 
[0.1, 0.9, 0],
[0.4, -0.5, 0],
[0.4, 0.6, 0],
[0.7, 0.2, 0], 
[0.1, 0.9, 0],
[0.4, -0.5, 0],
[0.4, 0.6, 0],
[0.7, 0.2, 0], 
[0.1, 0.9, 0],
[0.4, -0.5, 0],
[0.4, 0.6, 0],
[0.7, 0.2, 0], 
[0.1, 0.9, 0],
[0.4, -0.5, 0],
[0.4, 0.6, 0],
[0.7, 0.2, 0], 
[0.1, 0.9, 0],
[0.4, -0.5, 0],
[0.1, -0.7, 0],
[0.1, -0.5, 0]]  


class myNode:
    def __init__(self, *args):
        self.baseName = "base_link"
        self.poses = ['drive','0cm_Mid','0cm_Left','0cm_Right','5cm_Mid','5cm_Left','5cm_Right','10cm_Mid','10cm_Left','10cm_Right','15cm_Mid','15cm_Left','15cm_Right']
        rospy.init_node('mock_node', anonymous=True)
        self.TFbroadcaster = tf.TransformBroadcaster()


        s = rospy.Service('/pick', ManipulationAction, self.handle_pick)
        s = rospy.Service('/place', ManipulationAction, self.handle_place)
        s = rospy.Service('/move_camera', ManipulationPose, self.handle_move)

        s = rospy.Service('/get_scan_all', VisionScan, self.handle_scan)

	rospy.Service('move_to_goal', NavigationGoal, self.goal_handler)
        rospy.Service('reposition', NavigationRepose, self.repose_handler)
	rospy.Service('get_distance', DistanceToGoal, self.get_distance_handler)
	rospy.loginfo("Mock Ready")

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()

    def handle_pick(self,req):
        print("[UR3] pick from {}".format(req.link).rstrip("\n\r"))
        return ManipulationActionResponse(1)
    
    def handle_place(self, req):
        print("[UR3] place to {}".format(req.link).rstrip("\n\r"))
        return ManipulationActionResponse(1)
    
    def handle_move(self, req):
        print("[UR3] set arm to {}".format(self.poses[req.target]).rstrip("\n\r"))
        return ManipulationPoseResponse(True)

    def goal_handler(self, data):
	print("[DRIVE] drive to {}".format(data).rstrip("\n\r"))
	status = 1
        return status
        

    def repose_handler(self, data):
	print("[DRIVE] reposition to {}".format(data).rstrip("\n\r"))
	status = 1
        return status

    def get_distance_handler(self, waypoint_data):
	print("[DRIVE] get distance to {}".format(waypoint_data).rstrip("\n\r"))
        distance = 10
        return distance 
    
    def handle_scan(self, req):
        print("[VISION] scan for item")
        visionScanResponse = VisionScanResponse()
        visionScanResponse.result.append(VisionScanObject(1,_ids[1],1))
        visionScanResponse.result.append(VisionScanObject(2,"link2",1))
        visionScanResponse.result.append(VisionScanObject(3,"link3",1))
        visionScanResponse.result.append(VisionScanObject(4,"link4",1))
        visionScanResponse.result.append(VisionScanObject(5,"link5",1))
      	visionScanResponse.result.append(VisionScanObject(6,"link6",1))
        visionScanResponse.result.append(VisionScanObject(7,"link7",1))
        visionScanResponse.result.append(VisionScanObject(8,"link8",1))
      	visionScanResponse.result.append(VisionScanObject(9,"link9",1))
        visionScanResponse.result.append(VisionScanObject(10,"link10",1))
        visionScanResponse.result.append(VisionScanObject(11,"link11",1))
	visionScanResponse.result.append(VisionScanObject(12,"link12",1))
        visionScanResponse.result.append(VisionScanObject(13,"link13",1))
        visionScanResponse.result.append(VisionScanObject(14,"basketBlue",1))
        visionScanResponse.result.append(VisionScanObject(15,"basketRed",1))
	visionScanResponse.result.append(VisionScanObject(101,"link101",1))
        visionScanResponse.result.append(VisionScanObject(102,"link102",1))
        visionScanResponse.result.append(VisionScanObject(103,"link103",1))
	visionScanResponse.result.append(VisionScanObject(104,"link104",1))
        visionScanResponse.result.append(VisionScanObject(105,"link105",1))
        visionScanResponse.result.append(VisionScanObject(106,"link106",1))
	

        i = 0
        for x in visionScanResponse.result:
           self.addTF([_tfpos[i],[0,0,0,1]],"link{}".format((i+1)%100))
           i+=1 

        return visionScanResponse
    
    def addTF(self, pose , name):
        try:
            self.TFbroadcaster.sendTransform(
                translation=pose[0],
                rotation=pose[1],
                time=rospy.Time.now(),
                child=name,
                parent="base_link"
            )
            return True
        except tf.Exception:
            return False

if __name__ == '__main__':
    myNode()
