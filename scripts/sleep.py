#!/usr/bin/env python

################################################################

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage,Range,Imu
from geometry_msgs.msg import Twist,Pose

import miro_msgs
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state,core_control,core_config,bridge_config,bridge_stream

import opencv_apps
from opencv_apps.msg import CircleArrayStamped

import math
import numpy
import time
import sys
from miro_constants import miro

from datetime import datetime

class SleepMode():
    def __init__(self):
        self.eyelid_closure = 0.0
        self.lights_raw = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.tail = -1.0
        self.ear_rotate = [0.0,0.0]
        self.body_config = [0.0,0.0,0.0,0.0]
        self.body_config_speed = [0.0,0.0,-1.0,0.0]
        self.pub_platform_control = rospy.Publisher('miro/rob01/platform/control',platform_control,queue_size=0)
        
    def miro_sleep(self):
        q = platform_control()
        while not rospy.is_shutdown():
            q.eyelid_closure = 1.0
            q.lights_raw = [0,255,255,0,0,0,0,0,0,0,0,0,0,0,0,0,255,255]#white color
            q.tail = -1.0
            q.ear_rotate = [0.0,0.0]
            q.body_config = [0.0,0.0,0.0,0.0]
            q.body_config_speed = [0.0,-0.5,-0.5,-0.5]
            self.pub_platform_control.publish(q)

if __name__== '__main__':
    rospy.init_node('sleep')
    sleep = SleepMode()
    sleep.miro_sleep()