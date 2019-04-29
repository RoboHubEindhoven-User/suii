#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

class ClearCostmap(object):
    def __init__(self):
        rospy.init_node('ClearCostmap', anonymous=False)
        self.reset = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        rospy.Timer(rospy.Duration(secs=20, nsecs=0), callback=self.clear_costmap)
        rospy.spin()


    def clear_costmap(self):
        self.reset()
        

if __name__ == '__main__':
    try:
        ClearCostmap()
    except rospy.ROSInterruptException: 
        pass
        
