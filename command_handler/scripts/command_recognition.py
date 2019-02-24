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
        #self.tryd = False
        #------------------ ADD OBJECTS OF NEW COMMANDS ----------------------#

        ## Platform control Object that represents the sleep action
        self.q_sleep = platform_control()
        ## Platform control Object that represents the miro action when it is scolded ("Bad!")
        self.q_sad = platform_control()
        ## Platform control Object that represents the miro action when it is scolded ("Come!")
        self.q_follow = platform_control()
        ## Platform control Object that represents the miro action when it is scolded ("Good!")
        self.q_good = platform_control()
        ## platform_control object that rapresents the Gesture Based Behavior ('Dance')
        self.q_gbb = platform_control()

        ## Subscriber to the topic /activation a message of type Bool that notify commands activation
        #self.sub_activation = rospy.Subscriber('/activation',Bool,self.callback_activation,queue_size=1)
        ## Subscriber to the topic /speech_to_text a message of type String that cointains the vocal command converted in text
        self.sub_speech_to_text = rospy.Subscriber('/speech_to_text', String, self.callback_receive_command,queue_size=1)

        #------------------ ADD SUBSCRIBERS TO NEW COMMANDS -------------------#
        ## Subscriber to the topic /miro_sleep a message of type platform_control that rapresents the action corresponting to the command "Sleep"
        self.sub_sleep_action = rospy.Subscriber('/miro_sleep', platform_control, self.callback_sleep_action,queue_size=1)
        ## Subscriber to the topic /miro_sad a message of type platform_control that rapresents the action corresponting to the command "Bad"
        self.sub_sad_action = rospy.Subscriber('/miro_sad', platform_control, self.callback_sad_action,queue_size=1) 
        ## Subscriber to the topic /miro_follow a message of type platform_control that rapresents the action corresponting to the command "Follow"
        self.sub_follow_action = rospy.Subscriber('/miro_follow', platform_control, self.callback_follow_action,queue_size=1) 
        ## Subscriber to the topic /miro_dance a message of type platform_control that rapresents the action corresponting to the command "Dance
        self.sub_gbb = rospy.Subscriber('/gbb', platform_control, self.callback_gbb,queue_size=1) 
        ## Subscriber to the topic /miro_good a message of type platform_control that rapresents the action corresponting to the command "Good"
        self.sub_good_action = rospy.Subscriber('/miro_good', platform_control, self.callback_good_action,queue_size=1) 
        # ## Subscriber to the topic /miro_relax a message of type platform_control that rapresents the action corresponting to the command "Good"
        # self.sub_good_action = rospy.Subscriber('/miro_good', platform_control, self.callback_good_action,queue_size=1) 
       
        #self.sub_try = rospy.Subscriber('/try',Bool,self.callback_try,queue_size=1)

        ## Publisher to the topic /switch_off a message of type Bool which disable all the commands
        self.pub_sleep_mode = rospy.Publisher('/switch_off', Bool, queue_size=0)
        # ## Publisher to the topic /relax a message of type Bool which disable all the commands
        # self.pub_relax_mode = rospy.Publisher('/relax', Bool, queue_size=0) 
        ## Publisher to the topic /platform/control a message of type platform_control which execute Miro actions 
        self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control", platform_control, queue_size=0)
    
    ## Callback function that receives the enabling command
    # def callback_activation(self, activation):

    #     self.activate = activation
    
    ## Callback function that receive and save the user's voice command as text
    def callback_receive_command(self, text):

        self.command = text.data

    #------------------ ADD CALLBACK FOR NEW COMMANDS -------------------#    
    ## Callback that receives the action corresponding to vocal command "Sleep"
    def callback_sleep_action(self, sleep):

        self.q_sleep = sleep

    ## Callback that receives the action corresponding to vocal command "Sad"
    def callback_sad_action(self, sad):

        self.q_sad = sad
    
    ## Callback that receives the action corresponding to vocal command "Follow"
    def callback_follow_action(self, follow):

        self.q_follow = follow
    
    ## Callback that receives the Gesture Based Behavior as a platform_control message
    def callback_gbb(self,gbb):

        self.q_gbb = gbb
    
    ## Callback that receives the action corresponding to vocal command "Good"
    def callback_good_action(self, good):

        self.q_good = good
    
    # ## Callback that receives the action corresponding to vocal command "Relax"
    # def callback_good_action(self, relax):

    #     self.q_relax = relax
    #     # sensori?

    ## Function that check the incoming commands and, if the activation command ("Miro") is received, brings the robot in the default mode and enables the evaluation of futhers commands
    def switching_commands(self):

        q = platform_control()
        q.eyelid_closure = 1.0

        r = rospy.Rate(self.rate)
        count_bad = 0
        miao = 0
        count_miro = 0
        count_sleep = 0
        while not rospy.is_shutdown():

            #ACTIVATION COMMAND

            if self.command == "Miro" or self.command == " Miro" or self.command == "miro" or self.command == " miro":
                count_miro = 0
                count_miro = count_miro +1
                rospy.loginfo(count_miro)
                count_sleep = 0

                if count_miro == 1:
                    q.eyelid_closure = 0.0
                    q.lights_raw = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                    q.tail = -1.0
                    q.ear_rotate = [0.0,0.0]
                    q.body_config = [0.0,0.0,0.0,0.0]
                    q.body_config_speed = [0.0,-1.0,-1.0,-1.0]
                    self.pub_platform_control.publish(q)
                    
                self.activate = True

            # SLEEP
            if self.activate and self.command == "Sleep" or self.command == " Sleep" or self.command == "sleep" or self.command == " sleep":
                count_miro = 0
                count_bad = 0
                q = self.q_sleep
                self.pub_platform_control.publish(q)
                self.pub_sleep_mode.publish(True)
                self.activate = False
                count_sleep = 1
                print "Sleep"
                

            # BAD
            elif self.activate and (self.command == "Bad" or self.command == " Bad" or  self.command == "bad" or self.command == " bad"):
                count_bad = count_bad + 1
                rospy.loginfo(count_bad)
                if count_bad < 2000:
                    q = self.q_sad
                    self.pub_platform_control.publish(q)
                else:
                    q.body_vel.linear.x = 0.0
                    q.body_vel.angular.z = 0.0
                    q.lights_raw = [255,0,0,255,0,0,255,0,0,255,0,0,255,0,0,255,0,0]
                    self.pub_platform_control.publish(q)
                print "MIRO BAD"
            
            # FOLLOW
            elif self.activate and (self.command == "Come" or self.command == " Come" or self.command == "come" or self.command == " come"):
                count_bad = 0
                q = self.q_follow
                self.pub_platform_control.publish(q)
            # DANCE
            elif self.activate and (self.command == "Dance" or self.command == " Dance" or self.command == "dance" or self.command == " dance"):
                q = self.q_gbb
                print "Miro Dance"
            # GOOD
            elif self.activate and (self.command == "Good" or self.command == " Good" or self.command == "good" or self.command == " good"):
                count_bad = 0
                q = self.q_good
                self.pub_platform_control.publish(q)  
                print "Good"


            elif count_sleep == 1 and (not self.activate and not self.command == "Sleep"):
                rospy.loginfo(count_sleep)
                q.eyelid_closure = 1
                print "baubau"
                self.pub_platform_control.publish(q)

            r.sleep()
            # # RELAX
            # elif self.activate and (self.command == "Relax" or self.command == " Relax" or self.command == "relax" or self.command == " relax"):
           

if __name__== '__main__':

    rospy.init_node('command_recognition')
    sb = CommandRecognition()
    sb.switching_commands()