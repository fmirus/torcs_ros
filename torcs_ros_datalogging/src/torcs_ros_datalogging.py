#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Aug  9 15:52:05 2018

@author: bzorn
"""

import h5py
import yaml
import os
import rospy
import message_filters


from torcs_msgs.msg import TORCSSensors
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8, Float32
from std_msgs_stamped.msg import BoolStamped, Int8Stamped, Float32Stamped
from geometry_msgs.msg import TwistStamped, Vector3Stamped, PoseStamped

import sys
import rospkg
import datetime

cwd = rospkg.RosPack().get_path('torcs_ros_datalogging')
sys.path.append(cwd[:-21] + "common")
cwd = cwd[:-21]

from bzReadYAML import readTrajectoryParams, calcTrajectoryAmount, readNengoHyperparams, readConfigSrc, readsavedNengoHyperparams

class DataLogger:
    def __init__(self, cwd):
        #### various parameters and variables ####
        #choose index of scanners to use
        #angle min/max: +-1.57; increment 0.1653; instantenous, range: 200 m
        [_, _, _, self.a_selectScanTrack] = readNengoHyperparams(cwd)

        #can man sie im nur write öffnen? h5py Mutex problem
        #data structure
        self.data = dict( 
                ACTION_DATA = dict(action = 0, ref_time = 0),
                REWARD_DATA = dict(reward = 0, ref_time = 0), 
                LASER_DATA = dict(laser = len(self.a_selectScanTrack)*[0], ref_time = 0),
                SENSOR_DATA = dict(trackpos = 0, angle = 0, laptime = 0, distance = 0, ref_time = 0),
                GLOBAL_DATA = dict(posX = 0, posY = 0, ref_time= 0),
                SPEED_DATA = dict(speedX = 0, ref_time = 0)
                )
        
        #subscribers
        self.sub_action = rospy.Subscriber("/torcs_ros/TrajectorySelector", Int8Stamped, self.callback_action)
        self.sub_reward = rospy.Subscriber("/torcs_ros/logging/reward", Float32Stamped, self.callback_reward)
        self.sub_sensor = rospy.Subscriber("/torcs_ros/sensors_state", TORCSSensors, self.callback_sensors)
        self.sub_global = rospy.Subscriber("/torcs_ros/torcs_global_pose", PoseStamped, self.callback_global)
        self.sub_speed = rospy.Subscriber("/torcs_ros/speed", TwistStamped, self.callback_speed)
        
        self.createData();

    def callback_action(self, msg):
        self.data['ACTION_DATA']['action'] = msg.data
        self.data['ACTION_DATA']['ref_time'] = msg.header.stamp
    
    def callback_reward(self, msg):
        self.data['REWARD_DATA']['reward'] = msg.data
        self.data['REWARD_DATA']['ref_time'] = msg.header.stamp
    
    def callback_laser(self, msg):
        self.data['LASER_DATA']['action'] = msg.ranges
        self.data['LASER_DATA']['ref_time'] = msg.header.stamp

    def callback_sensors(self, msg):
        self.data['SENSOR_DATA']['angle'] = msg.angle
        self.data['SENSOR_DATA']['trackpos'] = msg.trackPos
        self.data['SENSOR_DATA']['laptime'] = msg.currentLapTime
        self.data['SENSOR_DATA']['distance'] = msg.distFromStart
        self.data['SENSOR_DATA']['ref_time'] = msg.header.stamp

    def callback_global(self, msg):
        self.data['GLOBAL_DATA']['posX'] = msg.pose.position.x
        self.data['GLOBAL_DATA']['posY'] = msg.pose.position.y
        self.data['GLOBAL_DATA']['ref_time'] = msg.header.stamp

    def callback_speed(self, msg):
        self.data['SPEED_DATA']['speedX'] = msg.twist.linear.x
        self.data['SPEED_DATA']['ref_time'] = msg.header.stamp

    def writeData(self, key1):
        for key2 in self.data[key1].keys():
            pass
#        pass
        
    def createData(self):
        self.log_file = '/home/bzorn/'
        success = True
        message = 'Successfully logged data to file ' + self.log_file
        try:
          with h5py.File(self.log_file, 'w') as hf:
              for key1 in self.data.keys():
                  for key2 in self.data[key1].keys():
                      if(type(self.data[key1][key2] == list)):
                          hf.create_dataset(key1+"_"+key2, (0,len(data[key1][key2])), maxshape=(None,len(data[key1][key2])), compression="gzip", compression_opts=9)
                      else:
                          hf.create_dataset(key1+"_"+key2, (0,), maxshape=(None,), compression="gzip", compression_opts=9)
              hf.close()
#        

        except:
          success = False
          message = 'Something went wrong during logging data to file ' + self.log_file
          print("wrong")
    
        return [success, message]
        


if __name__ == '__main__':
    rospy.init_node('datalogger') #initalize code
    rospy.wait_for_message("/torcs_ros/notifications/InputReceived", Bool)
    [data_save, _, _, _] = readConfigSrc(cwd)
    if(data_save):
        my_logger = DataLogger(cwd) #create instance
        
        rospy.spin()