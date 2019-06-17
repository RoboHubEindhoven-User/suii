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

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()

    def handle_pick(self,req):
        print("pick from {}".format(req.link))
        return ManipulationActionResponse(1)
    
    def handle_place(self, req):
        print("place to {}".format(req.link))
        return ManipulationActionResponse(1)
    
    def handle_move(self, req):
        print("move to {}".format(self.poses[req.target]))
        return ManipulationPoseResponse(True)
    
    def handle_scan(self, req):
        print("scan for item")
        visionScanResponse = VisionScanResponse()
        visionScanResponse.result.append(VisionScanObject(1,"link1",1))
        visionScanResponse.result.append(VisionScanObject(2,"link2",1))
        visionScanResponse.result.append(VisionScanObject(3,"link3",1))
        visionScanResponse.result.append(VisionScanObject(4,"link4",1))
        visionScanResponse.result.append(VisionScanObject(5,"link5",1))
        visionScanResponse.result.append(VisionScanObject(101,"link6",1))
        visionScanResponse.result.append(VisionScanObject(103,"link7",1))
        visionScanResponse.result.append(VisionScanObject(104,"link8",1))

        i = 0
        for x in visionScanResponse.result:
           self.addTF([_tfpos[i],[0,0,0,1]],"link{}".format(i+1))
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