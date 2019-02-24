#!/usr/bin/env python

################################################################

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage,Range,Imu
from geometry_msgs.msg import Twist,Pose
import random

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

class GoodMode():
    def __init__(self):
        ## Node rate
        self.rate = rospy.get_param('rate',200)
        self.pub_platform_control = rospy.Publisher('/miro_good',platform_control,queue_size=0)

    def miro_good(self):
        r = rospy.Rate(self.rate)
        q = platform_control()
        count = 0
        while not rospy.is_shutdown():
            q.eyelid_closure = 0.2
            q.sound_index_P1 = 1
            q.body_config = [0.0,0.25,0.0,-0.25]
            q.body_config_speed = [0.0,-1.0,-1.0,-1.0]
            q.lights_raw = [0,255,0,255,0,255,0,0,255,0,255,0,255,0,255,0,255,0]
            q.tail = 68
            self.pub_platform_control.publish(q)

            r.sleep()


if __name__== '__main__':
    rospy.init_node('good')
    good = GoodMode()
    good.miro_good()
