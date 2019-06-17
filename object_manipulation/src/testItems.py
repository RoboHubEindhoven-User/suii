#!/usr/bin/env python

import rospy
import tf

#services
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from suii_msgs.srv import UpdateItems, UpdateItemsResponse, UpdateItemsRequest
from suii_msgs.srv import Item, ItemResponse, ItemRequest
from suii_msgs.msg import VisionScanObject


tolerance = 10 * 0.01 # 10cm

class ItemObject:
    def __init__(self, id, link,presision):
        self.id = id
        self.link = link
        self.presision = presision

class itemTest:
    def __init__(self):
        rospy.init_node('testItems', anonymous=True)
        rospy.wait_for_service('items/updateItems')
        rospy.wait_for_service('items/clearItems')
        rospy.wait_for_service('items/moveItem')
        rospy.wait_for_service('items/getItem')
        rospy.wait_for_service('items/getHole')
        rospy.wait_for_service('items/removeItem')
        rospy.wait_for_service('items/printLists')

        self.updateItemsClient = rospy.ServiceProxy('items/updateItems', UpdateItems)
        self.clearItemsClient = rospy.ServiceProxy('items/clearItems', Empty)
        self.removeItemClient = rospy.ServiceProxy('items/removeItem',Item)
        self.moveItemClient = rospy.ServiceProxy('items/moveItem', Item)
        self.getItemClient = rospy.ServiceProxy('items/getItem',Item)
        self.getHoleClient = rospy.ServiceProxy('items/getHole',Item)
        self.printListsClient = rospy.ServiceProxy('items/printLists',Empty)
    

    def printList(self):
        #print
        try:
            self.printLists()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def updateList(self):
        updateItemsRequest = UpdateItemsRequest()
        updateItemsRequest.items.append(VisionScanObject(1,"link1",0))
        updateItemsRequest.items.append(VisionScanObject(2,"link2",0))
        updateItemsRequest.items.append(VisionScanObject(3,"link3",0))
        updateItemsRequest.items.append(VisionScanObject(4,"link4",0))
        updateItemsRequest.items.append(VisionScanObject(5,"link5",0))
        updateItemsRequest.items.append(VisionScanObject(101,"link6",0))
        updateItemsRequest.items.append(VisionScanObject(103,"link7",0))
        updateItemsRequest.items.append(VisionScanObject(104,"link8",0))
        #add items
        try:
            self.updateItemsClient(updateItemsRequest)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def removeItem(self, id,onRobot):
        try:
            self.removeItemClient(ItemRequest(id,onRobot))
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def moveItem(self, id, onRobot):
        try:
            self.moveItemClient(ItemRequest(id,onRobot))
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def getItem(self, id, onRobot):
        try:
            itemResponse =  self.getItemClient(ItemRequest(id,onRobot))
            print("##get object 1 from table")
            print(itemResponse)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def getHole(self,id):
        try:
            itemResponse = self.getHoleClient(ItemRequest(id,False))
            print("##get object 1 from table")
            print(itemResponse)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def clearList(self):
        try:
            self.clearItemsClient()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
 
    def test(self):
        self.updateList()
        self.moveItem(1,True)

if __name__ == '__main__':
    test = itemTest()
    test.test() 