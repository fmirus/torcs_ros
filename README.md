# Repo for interfacing TORCS with ROS

This is a ROS implementation of the client and driver from the [TORCS SCR C++ client](https://sourceforge.net/projects/cig/files/SCR%20Championship/Client%20C%2B%2B/). For the code in this repository to work you need a [patched version](https://github.com/fmirus/torcs-1.3.7) of [torcs1.3.7](https://sourceforge.net/projects/torcs/). This has only been tested with Ubuntu 16.04 and [ROS Kinetic](http://wiki.ros.org/kinetic)

## Installation 

 - Install the patched version of torcs1.3.7 according to its [installation instructions](https://github.com/fmirus/torcs-1.3.7)
 - Install [opencv](http://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html)
 - Install ROS according to the [installation insctructions](http://wiki.ros.org/kinetic/Installation/Ubuntu) and [create a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). 
 - Clone this repository into your catkin workspace and [build it](http://wiki.ros.org/catkin/commands/catkin_make)
 - 

## Usage

 - Run TORCS: ```torcs``` or ```torcs -noisy``` if you want noisy sensors (see the [SCR-Manual](https://arxiv.org/pdf/1304.1672.pdf) for details)
 - Run the ROS components in this repository: ```roslaunch torcs_ros_bringup torcs_ros.launch```. If you want to use your own driver instead of the simple driver in ```torcs_ros_drive_ctrl```, run ```roslaunch torcs_ros_bringup torcs_ros.launch driver:=false``` (make sure you remap your topics correctly).

## Description of individual packages

### torcs_img_publisher

this package publishes the current game image received via shared memory. For this package to work you need [opencv](http://opencv.org/)

### torcs_msgs

this package holds custom message files for torcs, namely ```TORCSCtrl``` and ```TORCSSensors```

### torcs_ros_bringup

this package holds config and launch files to start the whole ROS machinery

### torcs_ros_client

this is a ROS implementation of the original [SRC C++ client](https://sourceforge.net/projects/cig/files/SCR%20Championship/Client%20C%2B%2B/). However, this client only publishes data received from the game and subscribes to ctrl messages. It does not generate driving commands itself.

### torcs_ros_drive_ctrl

this is a separated implementation of the SimpleDriver contained in the original [SRC C++ client](https://sourceforge.net/projects/cig/files/SCR%20Championship/Client%20C%2B%2B/). It subscribes to the sensor messsage published by the ```torcs_ros_client```, generates simple drive commands and publishes them as ```TORCSCtrl``` messages.
