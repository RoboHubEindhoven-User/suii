#!/usr/bin/env python
import rospy
import tf
import numpy as np

#services server
from suii_msgs.srv import ItemPick, ItemPickResponse
from suii_msgs.srv import ItemPlace, ItemPlaceResponse
from suii_msgs.srv import ItemFindhole, ItemFindholeResponse
from suii_msgs.srv import ItemDrive, ItemDriveResponse

#vision
from suii_msgs.srv import VisionScan, VisionScanRequest, VisionScanResponse
#ur3
from suii_msgs.srv import ManipulationAction, ManipulationActionRequest, ManipulationActionResponse
from suii_msgs.srv import ManipulationPose, ManipulationPoseRequest, ManipulationPoseResponse
#ItemManager
from std_srvs.srv import Empty, EmptyResponse
from suii_msgs.srv import UpdateItems, UpdateItemsResponse, UpdateItemsRequest
from suii_msgs.srv import Item, ItemResponse, ItemRequest
from suii_msgs.srv import ItemMove, ItemMoveResponse, ItemMoveRequest

class objectManipulation:
    def __init__(self, *args):
        #UR3
        self.UR3_pick = rospy.ServiceProxy('/pick', ManipulationAction)
        self.UR3_place = rospy.ServiceProxy('/place', ManipulationAction)
        self.UR3_look = rospy.ServiceProxy('/move_camera', ManipulationPose)
        #Vision
        self.vision_scan = rospy.ServiceProxy('/get_scan_all', VisionScan)
        #ItemManager
        self.item_updateItems = rospy.ServiceProxy('items/updateItems', UpdateItems)
        self.item_clearItems = rospy.ServiceProxy('items/clearItems', Empty)
        self.item_removeItem = rospy.ServiceProxy('items/removeItem',Item)
        self.item_moveItem = rospy.ServiceProxy('items/moveItem', ItemMove)
        self.item_getItem = rospy.ServiceProxy('items/getItem',Item)
        self.item_getHole = rospy.ServiceProxy('items/getHole',Item)
        
        

    def drive(self):
        self.item_clearItems()
        self.UR3_look(ManipulationPoseRequest(1))
        pass
    
    def pick(self, link):
        self.UR3_look(ManipulationPoseRequest(1))
        self.UR3_pick(ManipulationActionRequest(link=link))

    def place(self,link):
        self.UR3_place(ManipulationActionRequest(link=link))

    def placeOnRobot(self,id):
        spot = 2
        name = "HOLDER_{0}".format(spot)
        self.place(name)
        request = ItemMoveRequest()
        request.itemID = id
        request.toRobot = True
        request.newLink = name
        self.item_moveItem(request)
        

    def placeOnTable(self,link):
        self.place("ONTABLE")

    def placeOnHole(self,link,id):
        hole = self.findHole()
        if hole is False:
            return False
        self.place(hole)
        return True

    def findHole(self,itemID):
        item = self.getHole(itemID)
        print(item)
        if item:
            return item
        i = 0
        while True:
            self.scan(i,1)
            item = self.getHole(itemID)
            print(item)
            if item:
                return item
            i+=1
            if i >= 3:
                break
        return False

    def findItem(self,itemID, onRobot):
        
        item = self.getItem(itemID,onRobot)
        print(item)
        if item:
            return item
        i = 0
        while True:
            self.scan(i,1)
            item = self.getItem(itemID,onRobot)
            print(item)
            if item:
                return item
            i+=1
            if i >= 3:
                break
        return False

    def getItem(self,itemID, onRobot):
        request = ItemRequest()
        request.itemID = itemID
        request.onRobot = onRobot
        response = self.item_getItem(request)
        if response.sucess:
            return response.link
        return False
    
    def getHole(self,itemID):
        request = ItemRequest()
        request.itemID = itemID
        request.onRobot = False
        response = self.item_getHole(request)
        if response.sucess:
            return response.link
        return False

    def scan(self, position, height ): # position: 1 mid, 2 left, 3 right;  
        #move to pose
        moveID = 1 + position + (height*3)
        self.UR3_look(ManipulationPoseRequest(moveID))
        #VisionScan
        visionResponse = self.vision_scan()
        #updateItemlists
        self.item_updateItems(UpdateItemsRequest(visionResponse.result))


    
class myNode:
    def __init__(self, *args):
        rospy.init_node('ObjectManipulation', anonymous=True)

        s1 = rospy.Service('ItemPick', ItemPick, self.handle_pick)
        s2 = rospy.Service('ItemPlace', ItemPlace, self.handle_place)
        s3 = rospy.Service('ItemFindhole', ItemFindhole, self.handle_findhole)
        s4 = rospy.Service('ItemDrive', ItemDrive, self.handle_drive)

        self.robot = objectManipulation()

    def handle_pick(self,req):   

        itemLink = self.robot.findItem(req.itemID, req.onRobot)
        if itemLink is False:
            return ItemPickResponse(sucess = False) 
        self.robot.pick(itemLink)
        return ItemPickResponse(sucess = True)

    def handle_place(self,req):
        response = ItemPlaceResponse(sucess = False)
        if req.onRobot:
            self.robot.placeOnRobot(req.itemID)
        else:
            if req.inHole:
                response.sucess = self.robot.placeOnHole(req.itemID)
            else:
                self.robot.placeOnTable("getItem")
        return response

    def handle_findhole(self,req):  
        hole = self.robot.findHole(req.itemID)
        if hole is False:
            return ItemFindholeResponse(sucess = False)
        return ItemFindholeResponse(sucess = True)

    def handle_drive(self,req):
        self.robot.drive()
        return ItemDriveResponse(sucess = True)

    def objectHandeler(self):
        

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    node = myNode()
    try:
        node.objectHandeler()
    except rospy.ROSInterruptException:
        pass