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


## \file oab_miro.py
## \brief The node oab_miro.py subscribes to the sonar and check the presence of an obstacle
## @n If the obstacle is present the strategy is to start turning in the direction given by the user through the smartwatch until no obstacle is present anymore.
## @n More in details: 
## @n Subscribe to the topic /platform/sensors
## @n Read the value from the sonar and check the presence of an obstacle
## @n if there is an obstacle implement the obstacle avoidance behaviour 
## @n The obstacle avoidance behavior:
## @n Subscribe to the topic /inertial
## @n save the last acceleration value
## @n if last acc[1]> 0 save the right
## @n if last acc[1]<0 save the left
## @n Then MiRo starts turnin in the direction of the last value saved 
## @n few degree each time the same obstacle is detected until there is no obstacle anymore

##The class ObstacleAvoidance implements the obstacle Avoidance Behavior

class BallDetection():

        
    def __init__(self):


        ## Signal if there is a Red Ball or not
        self.ball_dect = 0;
        self.count = 0;
        ## Subscriber to the topic /platform/sensors a message of type platform_sensors that cointains the sonar readings
        self.sub_ball = rospy.Subscriber('/hough_circles/circles', CircleArrayStamped, self.callback_ball,queue_size=1)
        ## Subscriber to the topic /imu_mapping a message of type Twist
        #self.sub_imu_data = rospy.Subscriber('/imu_mapping',Twist,self.callback_last_command,queue_size=1)

        #self.pub_platform_control = rospy.Publisher('/oab', platform_control, queue_size=0)

 
    def callback_ball(self, ball):

        self.ball_dect = ball.circles

	if not self.ball_dect:
		self.count = 0
		print "NO DETECTION"
	else:
		self.count = self.count + 1
		if self.count > 10:
			print "BALL DETECTED"


        
        #rospy.loginfo(sonar_msg.sonar_range)

    def main (self):
        rospy.spin()

if __name__== '__main__':
    rospy.init_node('ball_try')
    ball = BallDetection()
ball.main()
