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

class TurnLeft():
    def __init__(self):
        self.pub_platform_control = rospy.Publisher('miro/rob01/platform/control',platform_control,queue_size=0)

    def miro_turn_left(self):
        q = platform_control()
        while not rospy.is_shutdown():
            q.body_vel.linear.x = 0.0
            q.body_vel.angular.z = 1.4
            self.pub_platform_control.publish(q)

if __name__== '__main__':
    rospy.init_node('sleep')
    sleep = SleepMode()
    sleep.miro_sleep()