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

class BallDetection():

        
    def __init__(self):


        ## Signal if there is a Red Ball or not
        self.ball_right = False
        self.ball_left = False
        self.ball = False
        self.detect_right = 0
        self.detect_left = 0
        self.count_right = 0
        self.count_left = 0
        self.count_no_right = 0
        self.count_no_left = 0
        self.count_ball = 0
        self.radius_right = 0
        self.radius_left = 0
        ## Subscriber to the topic /platform/sensors a message of type platform_sensors that cointains the sonar readings
        self.sub_camera_right = rospy.Subscriber('right/hough_circles/circles', CircleArrayStamped, self.callback_camera_right,queue_size=1)
        ## Subscriber to the topic /imu_mapping a message of type Twist
        #self.sub_imu_data = rospy.Subscriber('/imu_mapping',Twist,self.callback_last_command,queue_size=1)
        self.sub_camera_left = rospy.Subscriber('left/hough_circles/circles', CircleArrayStamped, self.callback_camera_left,queue_size=1)
        self.pub_platform_control = rospy.Publisher('miro/rob01/platform/control',platform_control,queue_size=0)
        #self.pub_platform_control = rospy.Publisher('/oab', platform_control, queue_size=0)

 
    def callback_camera_right(self, ball):

        self.detect_right = ball.circles

        if not self.detect_right: 
            self.count_no_right = self.count_no_right + 1 
            if self.count_no_right > 5:
                self.count_right = 0
                print "NO DETECTION IN RIGHT CAMERA"
                self.ball_right = False
            
        else:
            self.count_right = self.count_right + 1
            if self.count_right > 10:
                print "BALL DETECTED IN RIGHT CAMERA"
                self.ball_right = True
                self.count_no_right = 0

    def callback_camera_left(self, ball):

        self.detect_left = ball.circles

        if not self.detect_left: 
            self.count_no_left = self.count_no_left + 1
            if self.count_no_left > 5:
                self.count_left = 0
                print "NO DETECTION IN LEFT CAMERA"
                self.ball_left = False
            
        else:
            self.count_left = self.count_left + 1
            if self.count_left > 10:
                print "BALL DETECTED IN LEFT CAMERA"
                self.ball_left = True
                self.count_no_left = 0
    
    def compared_detection(self):
        while not rospy.is_shutdown():

            if self.ball_right and self.ball_left:
                self.count_ball = self.count_ball + 1 
                if self.count_ball > 10:
                    self.ball = True
                    print "DETECTION COMPLETE"
            else:
                self.count_ball = 0
                self.ball = False
                print "NO COMPLETE DETECTION"

    



if __name__== '__main__':
    rospy.init_node('follow_me')
    ball = BallDetection()
    ball.compared_detection()


    