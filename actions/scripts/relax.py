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

class RelaxMode():
    def __init__(self):
        ## Node rate
        self.pub_platform_control = rospy.Publisher('/miro_relax',platform_control,queue_size=0)
        
    def miro_sleep(self):
        r = rospy.Rate(self.rate)
        q = platform_control()

        while not rospy.is_shutdown():
            
            r.sleep()


if __name__== '__main__':
    rospy.init_node('relax', disable_signals=True)
    sleep = RelaxMode()
    sleep.miro_relax()
