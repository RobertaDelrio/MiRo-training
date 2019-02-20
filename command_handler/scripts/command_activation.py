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

## \file command_activation.py 
## \brief The node command_activation.py allows to "wake up" Miro Robot and enables the recognition of further commands

## @n If an obstacle is detected then is used the information related to the user's comand to start turning in one direction of few degree.
## @n Each time the same obstacle is detected, the degrees respect to wich turns increases of a fixed quantity.
## @n This angular velocity and a red lightening pattern for Miro's body are used to construct the platform_control message to publish

class CommandActivation():

        
    def __init__(self):
        ## Node rate
        self.rate = rospy.get_param('rate',200)

        #topic root
        ## Allow to switch from real robot to simulation from launch file
        self.robot_name = rospy.get_param ( '/robot_name', 'rob01')
        topic_root = "/miro/" + self.robot_name
        print "topic_root", topic_root


        self.activation = False
        ## Initialization of the string to evaluate
        self.command = "string"
        self.count = 0
        ## Subscriber to the topic /speech_to_text a message of type String that cointains the vocal command converted in text
        self.sub_speech_to_text = rospy.Subscriber('/speech_to_text', String, self.callback_receive_command,queue_size=1)
        ## Subscriber to the topic /switch_off a message of type Bool that notify commands deactivation
        self.sub_sleep_mode = rospy.Subscriber('/switch_off',Bool,self.callback_switch_off,queue_size=1)
        ## Publisher to the topic /activation a message of type Bool which allow the execution of the others commands
        self.pub_activation = rospy.Publisher('/activation', Bool, queue_size=0)
        ## Publisher to the topic /platform/control a message of type platform_control which brings Miro in a default configuration
        self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control", platform_control, queue_size=0)

    ## Callback function that receive and save the user's voice command as text
    def callback_receive_command(self, text):

        self.command = text.data
        
    ## Callback that receives a message to deactivate the commands
    def callback_switch_off(self, switch_off):

        if switch_off:
            self.count = 0
            self.command = "string"
            
            

    ## Function that check the incoming commands and, if the activation command ("Miro") is received, brings the robot in the default mode and enables the evaluation of futhers commands
    def activate_commands(self):
        q = platform_control()
        r = rospy.Rate(self.rate)
        self.count = 0
        while not rospy.is_shutdown():
            if self.command == "Miro" or self.command == "miro" or self.command == " Miro" or self.command == " miro":
                self.count = self.count +1
                rospy.loginfo(self.count)
                if self.count == 1:
                    q.eyelid_closure = 0.0
                    q.lights_raw = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                    q.tail = -1.0
                    q.ear_rotate = [0.0,0.0]
                    q.body_config = [0.0,0.0,0.0,0.0]
                    q.body_config_speed = [0.0,-1.0,-1.0,-1.0]
                    self.pub_platform_control.publish(q)
                self.activation = True
                self.pub_activation.publish(True)
            r.sleep()
                


if __name__== '__main__':

    rospy.init_node('command_activation')
    sb = CommandActivation()
    sb.activate_commands()