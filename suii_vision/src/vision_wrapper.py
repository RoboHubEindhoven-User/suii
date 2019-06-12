#! /usr/bin/env python

import collections
import socket
from threading import Lock
import struct
import json

import rospy
import tf
from suii_msgs.srv import VisionScan
from suii_msgs.msg import VisionScanObject
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped

VisionMsg = collections.namedtuple("VisionMsg", ["id", "code", "body"])
_ids = {

}

class VisionClient(object):
    def __init__(self):
        self._sock = None
        self._shock_lock = Lock()

    def connect(self, address):
        try:
            self._shock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._shock.setblocking(True)
            self._shock.connect(address, 9001)
            return True
        except Exception:
            self._shock_lock.acquire()
            return False

    def process_one(self, function_id, list_args):
        if not self._shock_lock.acquire(False):
            return None
        d = self._encode(function_id, list_args)
        h, b = self._sync_data(d)
        r = self._decode(h, b)
        self._shock_lock.release()
        return r

    def _encode(self, function_id, list_args):
        if len(list_args) > 0:
            db = {}
            for x in list_args:
                db[x[0]] = x[1]
            j = json.dumps(db)
        else:
            j = ""
        sig = struct.pack('BBI', 0x00, function_id, len(j))
        return sig+j

    def _decode(self, header, body):
        dj = json.loads(body)
        r = VisionMsg(id=header[1], code=header[0], body=dj)
        return r

    def _sync_data(self, data):
        try:
            self._shock.sendall(data)
            rh = self._shock.recv(6)
            h = struct.unpack("BBI", rh)
            fragment = ""
            while len(fragment) < h[2]:
                fragment += self._shock.recv(h[2])
            return (h, fragment)
        except Exception:
            pass

class Wrapper(object):
    def __init__(self):
        self.client = VisionClient()
        if not self.client.connect("127.0.0.1"):
            rospy.logerr("Can't connect to server")

        rospy.init_node("vision_control", anonymous=False)
        rospy.Service("/get_scan_all", VisionScan, handler=self.callback_all)
        self.tfb = tf.TransformBroadcaster()
        self.tfl = tf.TransformListener()
        rospy.spin()

    def callback_all(self, service_data):
        result = self.client.process_one(0x00, [])
        if (result == None) or (result.code == 0xFF):
            return []

        rr = []
        for o in result.body:
            if o['t'] == 1:
                tr = self.addTF(o['x'], o['y'], o['z'], o['n'] + "_" + str(len(rr)))
            else:
                tr = False
            rm = VisionScanObject()
            rm.id = _ids[o['n']]
            if tr:
                rm.link = o['n'] + "_" + str(len(rr))
                rm.sure = 1
            else:
                rm.link = ""
                rm.sure = 0
            rr.append(rm)
        return rr

    def addTF(self, x, y, z, name):
        try:
            self.tfl.waitForTransform("camera", "ur3/base", rospy.Time(0), rospy.Duration(10))
            p = PoseStamped()
            p.header.frame_id = "camera"
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = z
            p.pose.orientation.w = 1.0
            pt = self.tfl.transformPose("ur3/base", p)
            self.tfb.sendTransform(
                translation=[pt.pose.position.x, pt.pose.position.y, pt.pose.position.z],
                rotation=[p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w],
                time=rospy.Time.now(),
                child=name,
                parent="ur3/base"
            )
            return True
        except tf.Exception:
            return False