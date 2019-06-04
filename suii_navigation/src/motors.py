#! /usr/bin/env python

import rospy
import time
import serial
import math

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from tf import TransformBroadcaster

class Motors(object):
    def __init__(self):
        rospy.init_node("Odrive_node")
        self.odrv0 = None
        self.odrv1 = None

        self.ready = False
        rospy.Subscriber("/cmd_vel_direct/right_front", Int32, callback=self.runDirectVel, callback_args=0)
        rospy.Subscriber("/cmd_vel_direct/left_front", Int32, callback=self.runDirectVel, callback_args=1)
        rospy.Subscriber("/cmd_vel_direct/right_back", Int32, callback=self.runDirectVel, callback_args=2)
        rospy.Subscriber("/cmd_vel_direct/left_back", Int32, callback=self.runDirectVel, callback_args=3)
        rospy.Subscriber("/cmd_vel", Twist, callback=self.runVel)

        self.stateEncoder = [0.0,0.0,0.0,0.0]
        self.stateZ = 0.0
        self.odom_msg = Odometry()
        self.odom_topic = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.tf_broadcast = TransformBroadcaster(queue_size=10)
        self.timestamp = time.time()
        rospy.Timer(rospy.Duration(secs=0, nsecs=20000), callback=self.getOdomDelta)

        rospy.Service("/motors/bootup", Empty, self.calibrateMotors)
        rospy.Service("/motors/reboot", Empty, self.rebootMotors)

        rospy.spin()

    def bootup(self):
        rospy.loginfo("ODrives loaded")
        time.sleep(1)
        self.odrv0.write("w axis0.requested_state 3\n")
        time.sleep(0.01)
        self.odrv0.write("w axis1.requested_state 3\n")
        time.sleep(0.01)
        self.odrv1.write("w axis0.requested_state 3\n")
        time.sleep(0.01)
        self.odrv1.write("w axis1.requested_state 3\n")
        time.sleep(2)
        self.checkState()

        time.sleep(0.01)
        self.odrv0.write("w axis0.requested_state 8\n")
        time.sleep(0.01)
        self.odrv0.write("w axis1.requested_state 8\n")
        time.sleep(0.01)
        self.odrv1.write("w axis0.requested_state 8\n")
        time.sleep(0.01)
        self.odrv1.write("w axis1.requested_state 8\n")
        time.sleep(0.01)

        self.odrv0.write("w axis0.controller.config.control_mode 2\n")
        time.sleep(0.01)
        self.odrv0.write("w axis1.controller.config.control_mode 2\n")
        time.sleep(0.01)
        self.odrv1.write("w axis0.controller.config.control_mode 2\n")
        time.sleep(0.01)
        self.odrv1.write("w axis1.controller.config.control_mode 2\n")
        time.sleep(0.01)
        rospy.loginfo("ODrives bootup done")

    def checkState(self):
        tryFlags = [True, True, True, True]
        while(tryFlags[0] or tryFlags[1] or tryFlags[2] or tryFlags[3]):
            if(tryFlags[0]):
                self.odrv0.write("r axis0.current_state\n")
                dat = self.odrv0.readline()
                if(dat == "1\r\n"):
                    tryFlags[0] = False
                time.sleep(0.1)
            if(tryFlags[1]):
                self.odrv0.write("r axis1.current_state\n")
                dat = self.odrv0.readline()
                if(dat == "1\r\n"):
                    tryFlags[1] = False
                time.sleep(0.1)
            if(tryFlags[2]):
                self.odrv1.write("r axis0.current_state\n")
                dat = self.odrv1.readline()
                if(dat == "1\r\n"):
                    tryFlags[2] = False
                time.sleep(0.1)
            if(tryFlags[3]):
                self.odrv1.write("r axis1.current_state\n")
                dat = self.odrv1.readline()
                if(dat == "1\r\n"):
                    tryFlags[3] = False
                time.sleep(0.1)
    
    def calibrateMotors(self, dat):
        self.odrv0 = serial.Serial("/dev/ttyODRV_F", 115200)
        self.odrv1 = serial.Serial("/dev/ttyODRV_B", 115200)
        time.sleep(2)
        self.odrv0.write("sb\n")
        self.odrv1.write("sb\n")
        time.sleep(5)
        self.odrv0 = serial.Serial("/dev/ttyODRV_F", 115200)
        self.odrv1 = serial.Serial("/dev/ttyODRV_B", 115200)
        time.sleep(2)
        self.bootup()
        self.ready = True
        return EmptyResponse()

    def rebootMotors(self, dat):
        if(self.ready):
            self.ready = False
            time.sleep(1)
            self.odrv0.write("sb\n")
            self.odrv1.write("sb\n")
            self.odrv0 = None
            self.odrv1 = None
            time.sleep(10)
            self.odrv0 = serial.Serial("/dev/ttyODRV_F", 115200)
            self.odrv1 = serial.Serial("/dev/ttyODRV_B", 115200)
            self.bootup()
            self.ready = True
	return EmptyResponse()

    def runDirectVel(self, dat, extra_arg):
        if(extra_arg == 0):
            self.odrv0.write("v 0 " + str(dat.data) + "\n")
        if(extra_arg == 1):
            self.odrv0.write("v 1 " + str(dat.data) + "\n")
        if(extra_arg == 2):
            self.odrv1.write("v 1 " + str(dat.data) + "\n")
        if(extra_arg == 3):
            self.odrv1.write("v 0 " + str(dat.data) + "\n")
    
    def runVel(self, dat):
        if(self.ready):
            pulse_count = 70000
            vec = [dat.linear.x/1.5, -dat.angular.z/4.5, dat.linear.y/1.5]
            qua = self.vectorToSpeed(vec)
            time.sleep(0.01)

            try:
                self.odrv0.write("v 0 " + str(int(qua[0]*pulse_count)) + "\n") #right front
                time.sleep(0.001)
                self.odrv0.write("v 1 " + str(-int(qua[1]*pulse_count)) + "\n") #left front
                time.sleep(0.001)
                self.odrv1.write("v 1 " + str(int(qua[2]*pulse_count)) + "\n") #right back
                time.sleep(0.001)
                self.odrv1.write("v 0 " + str(-int(qua[3]*pulse_count)) + "\n") #left back
                time.sleep(0.001)
            except serial.SerialException:
                rospy.loginfo("Odrive disconnected")
                self.ready = False
                self.odrv0 = None
                self.odrv1 = None
            except AttributeError:
                return

    def vectorToSpeed(self,vec):
        qua = [0.0, 0.0, 0.0, 0.0] 
        qua[0] = (vec[0] - vec[1] + vec[2]) #right front
        qua[1] = (vec[0] + vec[1] - vec[2]) #left front
        qua[2] = (vec[0] - vec[1] - vec[2]) #right back
        qua[3] = (vec[0] + vec[1] + vec[2]) #left back

        abs_mag = 0.0
        for x in qua:
            abs_mag = max(abs_mag, abs(x))
        if abs_mag > 1.0:
            for i in range(0, 4):
                qua[i] = (qua[i]/abs_mag)
        return qua
    
    def getOdomDelta(self, extra):
        if(self.ready):
            linear_constant = 0.000005532
            angular_constant = 0.000016984
            currentEnc = self.getEncoderData()
            if currentEnc is None:
                return
            deltaEnc = []
            for i in range(0,4):
                deltaEnc.append(currentEnc[i] - self.stateEncoder[i])
            self.stateEncoder = currentEnc
            delta_x = (deltaEnc[0] -deltaEnc[1] + deltaEnc[2] -deltaEnc[3])*linear_constant
            delta_y = (deltaEnc[0] + deltaEnc[1] - deltaEnc[2] -deltaEnc[3])*linear_constant
            tmp_z = self.stateZ
            self.stateZ += (deltaEnc[0] + deltaEnc[1] + deltaEnc[2] + deltaEnc[3])*angular_constant
            qua_r = quaternion_from_euler(0.0,0.0, self.stateZ)

            self.odom_msg.pose.pose.position.x += ((math.cos(self.stateZ)*delta_x) - (math.sin(self.stateZ)*delta_y))
            self.odom_msg.pose.pose.position.y += ((math.cos(self.stateZ)*delta_y) + (math.sin(self.stateZ)*delta_x))
            self.odom_msg.pose.pose.orientation.x = qua_r[0]
            self.odom_msg.pose.pose.orientation.y = qua_r[1]
            self.odom_msg.pose.pose.orientation.z = qua_r[2]
            self.odom_msg.pose.pose.orientation.w = qua_r[3]

            delta_time = time.time() - self.timestamp
            self.odom_msg.twist.twist.linear.x = delta_x/delta_time
            self.odom_msg.twist.twist.linear.y = delta_y/delta_time
            self.odom_msg.twist.twist.angular.z = (self.stateZ - tmp_z)/delta_time
            self.timestamp = time.time()

            self.odom_topic.publish(self.odom_msg)
            self.tf_broadcast.sendTransform(
                (self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y, 0.0),
                qua_r,
                rospy.Time.now(),
                "base_link",
                "odom"
            )
    def getEncoderData(self):
        encDat = [0.0,0.0,0.0,0.0]
        try:
            self.odrv0.write("r axis0.encoder.pos_estimate\n")
            tmp = self.odrv0.readline()
            encDat[0] = float(tmp[:-2])
            self.odrv0.write("r axis1.encoder.pos_estimate\n")
            tmp = self.odrv0.readline()
            encDat[1] = float(tmp[:-2])
            self.odrv1.write("r axis1.encoder.pos_estimate\n")
            tmp = self.odrv1.readline()
            encDat[2] = float(tmp[:-2])
            self.odrv1.write("r axis0.encoder.pos_estimate\n")
            tmp = self.odrv1.readline()
            encDat[3] = float(tmp[:-2])
        except serial.SerialException:
            rospy.loginfo("Odrive disconnected")
            encDat = None
            self.ready = False
            self.odrv0 = None
            self.odrv1 = None
	except AttributeError:
            encDat = None
        return encDat

if __name__ == "__main__":
    m = Motors()

