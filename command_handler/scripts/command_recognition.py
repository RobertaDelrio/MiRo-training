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

class CommandRecognition():

        
    def __init__(self):
        ## Node rate
        self.rate = rospy.get_param('rate',200)

        #topic root
        ## Allow to switch from real robot to simulation from launch file
        self.robot_name = rospy.get_param ( '/robot_name', 'rob01')
        topic_root = "/miro/" + self.robot_name
        print "topic_root", topic_root

        ## Initialization of the activation command
        self.activate = False
        ## Initialization of the string to evaluate
        self.command = "string"

        #------------------ ADD OBJECTS OF NEW COMMANDS ----------------------#

        ## Platform control Object that represents the sleep action
        self.q_sleep = platform_control()
        
        ## Subscriber to the topic /activation a message of type Bool that notify commands activation
        self.sub_activation = rospy.Subscriber('/activation',Bool,self.callback_activation,queue_size=1)
        ## Subscriber to the topic /speech_to_text a message of type String that cointains the vocal command converted in text
        self.sub_speech_to_text = rospy.Subscriber('/speech_to_text', String, self.callback_receive_command,queue_size=1)

        #------------------ ADD SUBSCRIBERS TO NEW COMMANDS -------------------#
        ## Subscriber to the topic /miro_sleep a message of type platform_control that rapresents the action corresponting to the command "Sleep"
        self.sub_sleep_action = rospy.Subscriber('/miro_sleep', platform_control, self.callback_sleep_action,queue_size=1)
          
        
        ## Publisher to the topic /switch_off a message of type Bool which disable all the commands
        self.pub_sleep_mode = rospy.Publisher('/switch_off', Bool, queue_size=0)
        ## Publisher to the topic /platform/control a message of type platform_control which execute Miro actions 
        self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control", platform_control, queue_size=0)
    
    ## Callback function that receives the enabling command
    def callback_activation(self, activation):

        self.activate = activation
    
    ## Callback function that receive and save the user's voice command as text
    def callback_receive_command(self, text):

        self.command = text.data
        
    ## Callback that receives a message to deactivate the commands
    def callback_sleep_action(self, sleep):

        self.q_sleep = sleep

    #------------------ ADD CALLBACK FOR NEW COMMANDS -------------------#

    ## Function that check the incoming commands and, if the activation command ("Miro") is received, brings the robot in the default mode and enables the evaluation of futhers commands
    def switching_commands(self):

        q = platform_control()
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():

            if self.activate and self.command == "Sleep" or self.command == "sleep" or self.command == " sleep"  :
                #self.pub_sleep_mode.publish(True)
                q = self.q_sleep
                self.pub_platform_control.publish(q)
                self.pub_sleep_mode.publish(True)
                self.activate = False
                    
            elif self.activate and self.command == "Dance":
                print "Miro Dance"

            r.sleep()

            """ if self.activate:
                r.sleep()
                if self.command == "Sleep":
                    self.activate = False
                    self.pub_sleep_mode.publish(True)
                    q = self.q_sleep
                    self.pub_platform_control.publish(q) """
                    
            """ if self.command == "Dance":
                    print "Miro Dance"
            r.sleep() """


                
                


if __name__== '__main__':

    rospy.init_node('command_recognition')
    sb = CommandRecognition()
    sb.switching_commands()