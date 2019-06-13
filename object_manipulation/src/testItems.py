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


def test():

    #     rospy.wait_for_service('add_two_ints')
    # try:
    #     add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
    #     resp1 = add_two_ints(x, y)
    #     return resp1.sum
    # except rospy.ServiceException, e:
    #     print "Service call failed: %s"%e

    rospy.init_node('testItems', anonymous=True)

    rospy.wait_for_service('items/updateItems')
    rospy.wait_for_service('items/clearItems')
    rospy.wait_for_service('items/moveItem')
    rospy.wait_for_service('items/getItem')
    rospy.wait_for_service('items/getHole')
    rospy.wait_for_service('items/removeItem')
    rospy.wait_for_service('items/printLists')

    updateItems = rospy.ServiceProxy('items/updateItems', UpdateItems)
    clearItems = rospy.ServiceProxy('items/clearItems', Empty)
    removeItem = rospy.ServiceProxy('items/removeItem',Item)
    moveItem = rospy.ServiceProxy('items/moveItem', Item)
    getItem = rospy.ServiceProxy('items/getItem',Item)
    getHole = rospy.ServiceProxy('items/getHole',Item)
    printLists = rospy.ServiceProxy('items/printLists',Empty)


    updateItemsRequest = UpdateItemsRequest()

    item = VisionScanObject()
    item.id = 1
    item.link = "link1"
    item.sure = 0
    updateItemsRequest.items.append(item)

    item = VisionScanObject()
    item.id = 2
    item.link = "link2"
    item.sure = 0
    updateItemsRequest.items.append(item)

    item = VisionScanObject()
    item.id = 3
    item.link = "link3"
    item.sure = 0
    updateItemsRequest.items.append(item)

    item = VisionScanObject()
    item.id = 101
    item.link = "link4"
    item.sure = 0
    updateItemsRequest.items.append(item)

    item = VisionScanObject()
    item.id = 102
    item.link = "link5"
    item.sure = 0
    updateItemsRequest.items.append(item)

    item = VisionScanObject()
    item.id = 103
    item.link = "link6"
    item.sure = 0
    updateItemsRequest.items.append(item)

    item = VisionScanObject()
    item.id = 104
    item.link = "link7"
    item.sure = 0
    updateItemsRequest.items.append(item)

    #add items
    try:
        updateItems(updateItemsRequest)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    #print
    try:
        printLists()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    #move to robot
    request = ItemRequest()
    request.itemID = 2
    request.onRobot = False
    try:
        moveItem(request)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    #print
    try:
        printLists()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    #get object 1 from table
    request = ItemRequest()
    request.itemID = 1
    request.onRobot = False
    try:
        itemResponse = getItem(request)
        print("##get object 1 from table")
        print(itemResponse)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
    #remove object 1 from table
    request = ItemRequest()
    request.itemID = 1
    request.onRobot = False
    try:
        removeItem(request)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    #print
    try:
        printLists()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
    #get object 2 from robot
    request = ItemRequest()
    request.itemID = 2
    request.onRobot = True
    try:
        itemResponse = getItem(request)
        print("##get object 2 from robot")
        print(itemResponse)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    #remove object 2 from robot
    request = ItemRequest()
    request.itemID = 2
    request.onRobot = True
    try:
        removeItem(request)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    #print
    try:
        printLists()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


    #get hole
    request = ItemRequest()
    request.itemID = 2
    try:
        itemResponse = getHole(request)
        print("##get hole")
        print(itemResponse)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    #clear list
    try:
        clearItems()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    #print
    try:
        printLists()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


    listener = tf.TransformListener()

    rate = rospy.Rate(10) # 10hz

if __name__ == '__main__':
    
    try:
        test()
    except rospy.ROSInterruptException:
        pass