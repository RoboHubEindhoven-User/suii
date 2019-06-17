#!/usr/bin/env python
import rospy
import tf
import numpy as np

#services server
from suii_msgs.srv import ItemPick, ItemPickResponse
from suii_msgs.srv import ItemPlace, ItemPlaceResponse
from suii_msgs.srv import ItemFindhole, ItemFindholeResponse
from suii_msgs.srv import ItemDrive, ItemDriveResponse

from std_msgs.msg import Float32
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
from suii_msgs.srv import getFreeSpot, getFreeSpotResponse, getFreeSpotRequest

class objectManipulation:
    def __init__(self, *args):
        #UR3
        self.UR3_pick = rospy.ServiceProxy('/pick', ManipulationAction)
        self.UR3_place = rospy.ServiceProxy('/place', ManipulationAction)
        self.UR3_look = rospy.ServiceProxy('/move_camera', ManipulationPose)
        #Vision
        self.vision_scan = rospy.ServiceProxy('/get_scan_all', VisionScan)
        #ItemManager
        self.item_getFreeSpot = rospy.ServiceProxy('items/getFreeSpot', getFreeSpot)
        self.item_updateItems = rospy.ServiceProxy('items/updateItems', UpdateItems)
        self.item_clearItems = rospy.ServiceProxy('items/clearItems', Empty)
        self.item_removeItem = rospy.ServiceProxy('items/removeItem',Item)
        self.item_moveItem = rospy.ServiceProxy('items/moveItem', ItemMove)
        self.item_getItem = rospy.ServiceProxy('items/getItem',Item)
        self.item_getHole = rospy.ServiceProxy('items/getHole',Item)

        rospy.Subscriber("/table_height", Float32, self.height_callback)

        self.TFbroadcaster = tf.TransformBroadcaster()

        self.dropIndex = 0
        self.tableHeight = 15.0
        
        
    def height_callback(self,data):
        self.tableHeight = data.data

    def drive(self): # make robot ready to drive
        self.dropIndex = 0
        try:
            self.item_clearItems() # clear table list
            self.UR3_look(ManipulationPoseRequest(0)) #set arm in drive position
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def pick(self, link,id,onRobot):
        self.UR3_pick(ManipulationActionRequest(link=link))
        if onRobot:
            self.item_removeItem(ItemRequest(id,onRobot)) # remove item from list back of robot


    def place(self,link):
        try:
            self.UR3_place(ManipulationActionRequest(link=link))
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def placeOnRobot(self,id): 
        response = self.item_getFreeSpot()
        if not response.sucess:
            return False
        spot = response.spot
        name = "HOLDER_{0}".format(spot)
        #print("place on robot: {0}".format(name))
        self.place(name) 
        self.item_moveItem(ItemMoveRequest(id, True, name))#id, toRobot, newlink
        return True
        

    def placeOnTable(self):
        
        x = -0.1 + (0.1*self.dropIndex) # 10cm between new drop points.(offset resets when driving.)
        y = -0.5 # 0.5 meter in front of robot (base_link)
        z = (self.tableHeight*0.01) + 0.04 # table height + 4 cm 

        pose = [[x,y,z],[0,0,0,1]]
        
        name = "table_{0}".format(self.dropIndex)
        self.addTF(pose,name)
        self.place(name)
        self.dropIndex +=1
        return True

    def placeOnHole(self,id):
        hole = self.findHole(id)
        if hole is False:
            return self.placeOnTable()
        self.place(hole)
        return True

    def findHole(self,itemID): # Find hole Behavior (curently mostly the same as findItem)
        item = self.getHole(itemID)
        if item:
            return item
        i = 0
        while True:
            self.scan(i,1)
            item = self.getHole(itemID)
            if item:
                return item
            i+=1
            if i >= 3:
                break
        return False

    def findItem(self,itemID, onRobot): # Find item Behavior:
        
        item = self.getItem(itemID,onRobot) # see if item in known before scanning
        #print(item)
        if item: # if it has a item return it.
            return item
        if onRobot: #if the item is`t found but it should be on the robot there is no point in looking for it on the table. return False
            return False

        #look for item on table
        scanHeight = int(self.tableHeight/5) #0cm->0, 5cm->1, 10cm->2, 15cm ->3
        i = 0
        while True:
            self.scan(i,scanHeight)
            item = self.getItem(itemID,onRobot)
            if item: #if item is found
                return item
            i+=1
            if i >= 3: # max 3 scans  
                break
        return False

    def getItem(self,itemID, onRobot):
        response = self.item_getItem(ItemRequest(itemID,onRobot)) # id, onRobot
        if response.sucess:
            return response.link
        return False
    
    def getHole(self,itemID):
        response = self.item_getHole(ItemRequest(itemID,False))# id, onRobot
        if response.sucess:
            return response.link
        return False

    def scan(self, position, height ): # position: 1 mid, 2 left, 3 right;  
        if position < 0 or position > 2:
            position = 0#if pose is not mid(0), left(1), right(2) set to mid(0)
        if height < 0 or height > 3:
            height = 3# if heitght is wrong set to max height (3->15cm)
        moveID = 1 + position + (height*3)
        self.UR3_look(ManipulationPoseRequest(moveID)) #move to scan pose.
        visionResponse = self.vision_scan()#let vision look for items.
        self.item_updateItems(UpdateItemsRequest(visionResponse.result)) #update the list with known Items

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


    
class myNode:
    def __init__(self, *args):
        rospy.init_node('ObjectManipulation', anonymous=True)

        s1 = rospy.Service('ItemPick', ItemPick, self.handle_pick)
        s2 = rospy.Service('ItemPlace', ItemPlace, self.handle_place)
        s3 = rospy.Service('ItemFindhole', ItemFindhole, self.handle_findhole)
        s4 = rospy.Service('ItemDrive', ItemDrive, self.handle_drive)

        self.robot = objectManipulation()

    def handle_pick(self,req):   
        print("pick called {}".format(req))
        itemLink = self.robot.findItem(req.itemID, req.onRobot)
        if itemLink is False:
            return ItemPickResponse(sucess = False) 
        self.robot.pick(itemLink,req.itemID,req.onRobot)
        return ItemPickResponse(sucess = True)

    def handle_place(self,req):
        print("place called {}".format(req))
        response = ItemPlaceResponse(sucess = False)
        if req.onRobot:
            response.sucess = self.robot.placeOnRobot(req.itemID)
        else:
            if req.inHole:
                response.sucess = self.robot.placeOnHole(req.itemID)
            else:
                response.sucess = self.robot.placeOnTable()
        return response

    def handle_findhole(self,req):  
        print("findhole called")
        hole = self.robot.findHole(req.itemID)
        if hole is False:
            return ItemFindholeResponse(sucess = False)
        return ItemFindholeResponse(sucess = True)

    def handle_drive(self,req):
        print("drive called")
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