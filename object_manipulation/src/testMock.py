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
    "Red_Container_Top",
    "Red_Container_Front",
    "Blue_Container_Top",
    "Blue_Container_Front"
]
_idsMall = [
    "F20_20_Mall",
    "F40_40_Mall",
    "R20_Mall",
    "Bolt_Mall",
    "M20_Mall",
    "M30_Mall"
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
        self.poses = ['drive','0cm_Mid','0cm_Left','0cm_Right','4cm_Mid','4cm_Left','4cm_Right','9cm_Mid','9cm_Left','9cm_Right','16cm_Mid','16cm_Left','16cm_Right','23cm_Mid','23cm_Left','23cm_Right']
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
	self.scanpose = 0

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
	if req.target == 0:
		print("[UR3] set arm to {}".format(self.poses[req.target]))
	else:
		self.scanpose = (req.target-1)%3
		print("[UR3] set arm to {} {}".format(self.poses[req.target],self.scanpose))
		
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
	
	if self.scanpose == 0:
		for x in range(0,5):
			visionScanResponse.result.append(VisionScanObject(x+1,_ids[x],1))
	if self.scanpose == 1:
		for x in range(5,10):
			visionScanResponse.result.append(VisionScanObject(x+1,_ids[x],1))
	if self.scanpose == 2:
		for x in range(10,len(_ids)):
			visionScanResponse.result.append(VisionScanObject(x+1,_ids[x],1))

	visionScanResponse.result.append(VisionScanObject(101,_idsMall[0],1))
        visionScanResponse.result.append(VisionScanObject(103,_idsMall[1],1))
        visionScanResponse.result.append(VisionScanObject(108,_idsMall[2],1))
	visionScanResponse.result.append(VisionScanObject(105,_idsMall[3],1))
        visionScanResponse.result.append(VisionScanObject(106,_idsMall[4],1))
        visionScanResponse.result.append(VisionScanObject(107,_idsMall[5],1))
	

        i = 0
        for x in visionScanResponse.result:
           self.addTF([_tfpos[i],[0,0,0,1]],x.link)
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
