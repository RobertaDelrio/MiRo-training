 # Gesture Based Control of MiRo Robot

 ## The Project
 This Project has been developed for the Social Robotics course of the master degree program in Robotics Engineering at University of Genoa.

 ### MiRo Companion Robot
 MiRo is a animal-like robot developed as a prototype companion.
 It was designed with a bio-inspired architecture based on the neuroscience knowledge of the mammalian brain.

 ### The wearable device
 In order to interact with MiRo through gestures, a smartwatch with a 9-axis IMU sensor has been used.
 [LG G WATCH R](https://www.lg.com/wearable-technology/lg-G-Watch-R-W110)

 ### The architecture


 ## The implementation 

 # Gettin Started

 ## Prerequisites

 ### ROS
This project is developed using [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu):
* rosdistro: kinetic
* rosversion: 1.12.13

### Smartwatch and Smartphone Setup
In order to publish imu sensor data from your smartwatch to ROS nodes you must have a smartwatch paired with a smartphone.
The smartphone acts as the bridge between the smartwatch and the ros master running on your computer.

Follow the instructions reported in [imu_stream](https://github.com/EmaroLab/imu_stream) to download the app for both the smartphone and the smartwatch.

### MQTT ROS Bridge

In order to succesfully subscribe to MQTT topics and publish contents of MQTT messages to ROS follow the instruction in [mqtt_ros_bridge](https://github.com/EmaroLab/mqtt_ros_bridge/tree/feature/multiple_smartwatches).
To work with the current project some parameter must be modified in the imu_bridge.launch 
The parameter device_name must be changed with the name of your personal smartwatch. 

### MiRo Workstation Setup
Download the [Miro Developer kit](http://labs.consequentialrobotics.com/miro/mdk/).

Follow the instructions from Consequential Robotics [Miro: Prepare Workstation](https://consequential.bitbucket.io/Developer_Preparation_Prepare_workstation.html) to set up your workstation to work with miro. 
Strictly follow the instructions in the Install MDK section as the following steps will rely on this.

Not necessary to make static IP for your workstation (laptop) while setting up connection with MiRo.

### Gesture Based Control MiRo

Create a catkin workspace and clone all the packages in the src folder

```
$ git clone https://github.com/EmaroLab/MiRo-training.git
$ cd MiRo-training
$ catkin_make
$ source devel/setup.bash
```

## Run the Project

Open a new terminal and launch

```
$ roscore
```
mosquitto must be running on your PC for the birdge to work.

In a new terminal
```
$ mosquitto
```
Make sure that the IP in the IMU_stream app on the smartphone is the same shown by doing

```
$ ifconfig
```

Open the IMU_stream app on the smartwatch 

Launch the mqtt_bridge in a new terminal
```
$ cd catkin_ws
$ cd source devel/setup.bash
$ roslaunch mqtt_bridge imu_bridge.launch
```
To test if the connection between smartwatch and ROS is working start to transmitt the data from IMU_stream app on the smartwatch and check in a new terminal
```
$ rostopic echo \imu_left_hand
```
It should show the Imu data published by the smartwatch.

Connect the Miro robot to the ROS Master

```
$ ssh root@<MIRO-IP> 
$ sudo nano ./profile
```
Insert your IP after ROS_MASTER_IP

For more detailed instructions see [MIRO: Commission MIRO](https://consequential.bitbucket.io/Developer_Preparation_Commission_MIRO.html)

The following command will start the project



## Acknowledgments

* [mqtt_ros_bridge](https://github.com/EmaroLab/mqtt_ros_bridge) 
* [imu_stream](https://github.com/EmaroLab/imu_stream)


### Team
* Roberta Delrio *roberta.delrio@studio.unibo.it*
* Valentina Pericu *valentina.pericu.@gmail.com*
