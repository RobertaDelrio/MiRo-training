#!/usr/bin/env python

################################################################ 

## \file dance.py
## \brief The node dance.py subscribes to the smartwatch's accelerometer data and publish the linear and angular velocities corresponding to some specific Motion of the Robot.
## @n More in details:
## @n It subscribes to the topic /inertial
## @n It reads from that topic the values of linear accelerations along x and y
## @n It maps these accelerations into linear and angular velocities 
## @n It publish on /imu_mapping the mapped values
## @n Two modality are available: BASE and ADVANCED
## and they can be selected from launch file



import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage,Range,Imu
from geometry_msgs.msg import Twist,Pose


import math
import numpy
import time
import sys
from miro_constants import miro

from datetime import datetime

## \brief The Class SmartwatchData allows the subscription to the smartwatch's accelerometer data and publishing of the 
## linear and angular velocities conveniently mapped.


class DanceGesture():

    ##Constructor
    def __init__(self):

        ##Last acelleration data received from smartwatch
        self.last_acc = [0,0,0]
        ##Last gyro velocity received from smartwatch
        self.g = 0
        ##Subscriber to the topic /inertial a message of type Imu
        self.sub_smartwatch = rospy.Subscriber('/inertial',Imu,self.callback_smartwatch_data,queue_size=1)
        ##Publisher on the topic /imu_mapping a message of type Twist
        self.pub_mapping = rospy.Publisher('/miro_dance', Twist, queue_size=0)
    
    ## Callback function that when the data from the smartwatch are received maps them into linear and angular velocities.
    ## Based on the value of the attribute mode, two different modalities of control are available
    ## @n <b>mode: basic</b>
    ## <div style="margin-left:40px;">
    ## It allows the user to exert three different types of control. 
    ## They correspond to three gesture and therefore to specific ranges of linear acceleration values.
    ## @n The control gesture are:
    ## <ul><li><i>Stay Still</i></li> 
    ## <li><i>Turn Right/Left</i></li>
    ## <li><i>Go Forward/Backward</i></li></ul></div>
    ## @n <b>mode: advanced</b>
    ## <div style="margin-left:40px;">
    ## It allows the user to exert the basic types of control but also combination of them. 
    ## @n All the linear acceleration values beetween the basic ranges are mapped as combination of linear and angular velocities.
    ## @n<i> e.g The control gesture combination could be Go Forward and Turn Right.</i>  </div>
    def callback_smartwatch_data(self,imu_data):
        
        self.last_acc[0] = imu_data.linear_acceleration.x
        self.last_acc[1] = imu_data.linear_acceleration.y
        self.last_acc[2] = imu_data.linear_acceleration.z

        self.g = imu_data.angular_velocity.z
        self.lastyaw = False
    def gesture_decoding (self):

        r = rospy.Rate(self.rate)
        q = platform_control()
        count = 0
        while not rospy.is_shutdown():

            if self.g > 2:

                self.lastyaw = True

            else:

                if self.lastyaw:

                    count = count + 1

                    self.lastyaw = False
            
            
           rospy.loginfo(count)


            self.pub_platform_control.publish(q)

            r.sleep()


if __name__== '__main__':
    rospy.init_node('dance')
    good = DanceGesture()
    good.gesture_decoding()
