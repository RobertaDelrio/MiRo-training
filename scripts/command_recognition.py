#!/usr/bin/env python

################################################################

import rospy
from std_msgs.msg import String,Bool,Int32
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

class CommandRecognition():

        
    def __init__(self):

        ## Signal if there is an obstacle or not
        self.command = "string"
        
        ## Subscriber to the topic /platform/sensors a message of type platform_sensors that cointains the sonar readings
        self.sub_sonar_data = rospy.Subscriber('/speech_to_text', String, self.callback_receive_command,queue_size=1)
        ## Subscriber to the topic /imu_mapping a message of type Twist
        self.sub_sleep_mode = rospy.Subscriber('/switch_off',Bool,self.callback_switch_off,queue_size=1)
        ## Publisher to the topic /oab a message of type platform_control which corresponds to the Obstacle Avoidance Behavior
        self.pub_activation = rospy.Publisher('/command', Int32, queue_size=0)

    ## Callback function that receive and save the user's voice command as text
    def callback_receive_command(self, text):

        self.command = text.data
        
    ## Callback that receives the data from the robot sensors, and uses the information given by the sonar sensor to evaluate the presence of an obstacle.
    ## @n If an obstacle is detected then is used the information related to the user's comand to start turning in one direction of few degree.
    ## @n Each time the same obstacle is detected, the degrees respect to wich turns increases of a fixed quantity.
    ## @n This angular velocity and a red lightening pattern for Miro's body are used to construct the platform_control message to publish

    
    def callback_switch_off(self, switch_off):

        if switch_off:
            self.command = "string"

    def compare_commands(self):
        while not rospy.is_shutdown():
            if self.command == "Miro":
                self.pub_activation.publish(1)


if __name__== '__main__':

    rospy.init_node('command_recognition')
    sb = CommandRecognition()
    sb.compare_commands()