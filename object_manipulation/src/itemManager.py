#!/usr/bin/env python

import rospy
import tf
import numpy as np

#services
from std_srvs.srv import Empty, EmptyResponse
from suii_msgs.srv import UpdateItems, UpdateItemsResponse
from suii_msgs.srv import Item, ItemResponse
from suii_msgs.srv import ItemMove, ItemMoveResponse, ItemMoveRequest

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped



tolerance = 10 * 0.01 # 10cm

class ItemObject:
    def __init__(self, id, link,presision):
        self.id = id
        self.link = link
        self.presision = presision
        self.pose = Pose()

class myNode:
    def __init__(self, *args):
        self.onTableList = []
        self.onBackList = []
        self.HoleList = []
        self.baseName = "base_link"

    def addTF(self, pose , name):
        print(pose[0])
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

    def isSame(self,tf1,tf2): #calculate distance between two tf frames and determine if they are closs enough to be the same object 
        distance = 0.0
        try:
            (trans,rot) = self.TFlistener.lookupTransform(tf1,tf2, rospy.Time(0))
            distance = np.linalg.norm(trans)
            rospy.loginfo("Distance between the hands is = {0:f}".format(distance))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("failed dis transform")
        
        
        if distance > tolerance:
            return False
        return True

    def findInList(self,item,itemsList,cmp):
        i = 0
        for compareItem in itemsList:
            #print("itemID  item:{0}, compareItem: {1}".format(item.id,compareItem.id))
            if item.id == compareItem.id:
                #print("simulare found item in list {0}".format(i))
                if not cmp:
                    return i
                if self.isSame(item.link,compareItem.link):
                    #print("is same object")
                    return i
            i += 1
        return -1

    def updateList(self,item,targetList):
        try:
            item.pose = self.TFlistener.lookupTransform(self.baseName,item.link, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("failed transform")
        listIndex = self.findInList(item,targetList,True)
        if listIndex == -1: # new item, add to list 
            targetList.append(item)
        else: # known item, update in list.
            targetList[listIndex] = item

    def handle_updateItems(self,req):
        rospy.loginfo("update itemList")
        for listItem in req.items:
            #print("update Item:")
            #print(listItem)
            item = ItemObject(listItem.id,listItem.link,listItem.sure) # convered msg to item object
            if item.presision == 0:
                continue
            if item.id < 100: # if the item id is less then 100 its a item oterwise it`s a hole 
                self.updateList(item,self.onTableList)
            else:
                item.id = item.id % 100
                #print("hole with id: {0}".format(item.id))
                self.updateList(item,self.HoleList)            

        return UpdateItemsResponse(True)

    def handle_clearItems(self,req): # clear list before driving with robot
        rospy.loginfo("Clearing itemList")
        del self.onTableList[:] #clear ontable list 
        del self.HoleList[:] #clear ontable list 
        return EmptyResponse()

    def handle_moveItem(self,req):
        rospy.loginfo("move Item: {0} to Robot: {1}".format(req.itemID,req.toRobot))
        response = ItemMoveResponse(True)

# uint16 itemID
# bool toRobot
# string newLink

        if not req.toRobot:
            sourceList = self.onBackList
            targetList = self.onTableList
        else:
            sourceList = self.onTableList
            targetList = self.onBackList
            if len(self.onBackList) >= 3: # max 3 objects on back of robot
                print("robot full")
                response.sucess = False
                return response

        item = ItemObject(req.itemID,"",0)
        listIndex = self.findInList(item,sourceList,False)
        item = sourceList[listIndex]
        try:
            item.pose = self.TFlistener.lookupTransform(self.baseName,req.newLink, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("failed transform")
        targetList.append(item)
        del sourceList[listIndex] 
        self.addTF(item.pose,item.link)   
        return response

    def handle_getItem(self,req):
        rospy.loginfo("get Item")
        response = ItemResponse(True,0,"")
        item = ItemObject(req.itemID,"",0)

        if req.onRobot:
            sourceList = self.onBackList
        else:
            sourceList = self.onTableList

        listIndex = self.findInList(item,sourceList,False)
        if listIndex == -1:
            response.sucess = False
            return response
        response.id = sourceList[listIndex].id
        response.link = sourceList[listIndex].link
        #response.link = "getItem"
        self.addTF(sourceList[listIndex].pose,response.link)
        return response

    def handle_getHole(self,req):
        rospy.loginfo("get Hole")
        response = ItemResponse(True,0,"")
        item = ItemObject(req.itemID,"",0)

        listIndex = self.findInList(item,self.HoleList,False)
        if listIndex == -1:
            response.sucess = False
            return response
        response.id = self.HoleList[listIndex].id
        response.link = self.HoleList[listIndex].link
        # response.link = "getHole"
        self.addTF(self.HoleList[listIndex].pose,response.link)
        return response

    def handle_removeItem(self,req):
        rospy.loginfo("remove item")
        response = ItemResponse(True,0,"")

        if req.onRobot:
            sourceList = self.onBackList
        else:
            sourceList = self.onTableList

        item = ItemObject(req.itemID,"",0)
        listIndex = self.findInList(item,sourceList,False)
        if listIndex == -1:
            response.sucess = False
            return response
        del sourceList[listIndex]
        return response

    def handle_printLists(self,req): 
        rospy.loginfo("print list") 
        print("##########################")
        print("onTableList size: {0}".format(len(self.onTableList)))
        for listItem in self.onTableList:
            print(" item: id:{0}, link:{1}, presision:{2} ".format(listItem.id,listItem.link,listItem.presision))
            self.addTF(listItem.pose,listItem.link)   
        print("onBackList size: {0}".format(len(self.onBackList)))
        for listItem in self.onBackList:
            print(" item: id:{0}, link:{1}, presision:{2} ".format(listItem.id,listItem.link,listItem.presision))
            self.addTF(listItem.pose,listItem.link)   
        print("HoleList size: {0}".format(len(self.HoleList)))
        for listItem in self.HoleList:
            print(" item: id:{0}, link:{1}, presision:{2} ".format(listItem.id,listItem.link,listItem.presision))
            self.addTF(listItem.pose,listItem.link)
        print("##########################")
        return EmptyResponse()

    def itemHandeler(self):
        s = rospy.Service('items/updateItems', UpdateItems, self.handle_updateItems)
        s = rospy.Service('items/clearItems', Empty, self.handle_clearItems)
        s = rospy.Service('items/printLists', Empty, self.handle_printLists)
        s = rospy.Service('items/moveItem', ItemMove, self.handle_moveItem)
        s = rospy.Service('items/getItem', Item, self.handle_getItem)
        s = rospy.Service('items/getHole', Item, self.handle_getHole)
        s = rospy.Service('items/removeItem', Item, self.handle_removeItem)
        

        rospy.init_node('itemManager', anonymous=True)
        self.TFlistener = tf.TransformListener()
        self.TFbroadcaster = tf.TransformBroadcaster()

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    node = myNode()
    try:
        node.itemHandeler()
    except rospy.ROSInterruptException:
        pass