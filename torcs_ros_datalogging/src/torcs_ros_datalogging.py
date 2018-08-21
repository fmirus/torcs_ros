#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Aug  9 15:52:05 2018

@author: bzorn
"""

import h5py
from mpi4py import MPI #used for parallel access of h5py
import yaml
import os
import rospy
import message_filters


from torcs_msgs.msg import TORCSSensors, TORCSCtrl
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8, Float32
from std_msgs_stamped.msg import BoolStamped, Int8Stamped, Float32Stamped
from geometry_msgs.msg import TwistStamped, PoseStamped

import sys
import rospkg
import datetime

cwd = rospkg.RosPack().get_path('torcs_ros_datalogging')
#cwd = "/home/bzorn/torcs/catkin_ws/src/torcs_ros/torcs_ros_datalogging"

sys.path.append(cwd[:-21] + "common")

cwd = cwd[:-21]

from bzReadYAML import readTrajectoryParams, calcTrajectoryAmount, readNengoHyperparams, readConfigSrc, readsavedNengoHyperparams

class DataLogger:
    def __init__(self, cwd, save_directory):
        [_, _, _, self.a_selectScanTrack] = readNengoHyperparams(cwd)
        self.save_directory = save_directory
        self.b_SaveOtherData = True
        self.n_StartingPoint = -1
        #can man sie im nur write öffnen? h5py Mutex problem
        #data structure
        tRefTime = dict(secs = 0, nsecs = 0)
        self.data = dict( 
                ACTION_DATA = dict(action = 0, ref_time = tRefTime),
                REWARD_DATA = dict(reward = 0, ref_time = tRefTime), 
                LASER_DATA = dict(laser = len(self.a_selectScanTrack)*[0], ref_time = tRefTime),
                SENSOR_DATA = dict(trackpos = 0, angle = 0, laptime = 0, distance = 0, ref_time = tRefTime),
                GLOBAL_DATA = dict(posX = 0, posY = 0, ref_time= tRefTime),
                SPEED_DATA = dict(speedX = 0, ref_time = tRefTime),
                CTRL_DATA = dict(meta = 0, ref_time = tRefTime),
                TRAJECTORY_DATA = dict(startingPoint = 0, ref_time = tRefTime)
                )
        self.createData();
        self.mutex = False;
#        #subscribers
        self.sub_action = message_filters.Subscriber("/torcs_ros/TrajectorySelector", Int8Stamped)#, self.callback_action)
        self.sub_reward = rospy.Subscriber("/torcs_ros/logging/reward", Float32Stamped, self.callback_reward, queue_size=1)
        self.sub_laser = message_filters.Subscriber("/torcs_ros/scan_track", LaserScan)#, self.callback_laser)
        self.sub_sensor = rospy.Subscriber("/torcs_ros/sensors_state", TORCSSensors, self.callback_sensors, queue_size=1)
        self.sub_global = rospy.Subscriber("/torcs_ros/torcs_global_pose", PoseStamped, self.callback_global, queue_size=1)
        self.sub_speed = rospy.Subscriber("/torcs_ros/speed", TwistStamped, self.callback_speed, queue_size=1)
        self.sub_ctrl = rospy.Subscriber("/torcs_ros/ctrl_cmd", TORCSCtrl, self.callback_ctrl, queue_size=1)
        self.sub_startingPoint = rospy.Subscriber("/torcs_ros/StartingPoint", Int8Stamped, self.callback_startingPoint, queue_size=1)
        self.sub_dontSave = rospy.Subscriber("/torcs_ros/notifications/dontSave", BoolStamped, self.callback_dontSave, queue_size=1)

        self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.sub_action, self.sub_laser], 2, 0.2)
        #### callback functions ####
        self.synchronizer.registerCallback(self.callback_action)
        self.synchronizer.registerCallback(self.callback_laser)
        self.write_ctrl = False

        
    def callback_action(self, msg_action, msg_laser):
        self.data['ACTION_DATA']['action'] = msg_action.data
        self.headerToDict('ACTION_DATA', msg_action.header)
        self.writeData('ACTION_DATA')
    
    def callback_reward(self, msg):
        self.data['REWARD_DATA']['reward'] = msg.data
        self.headerToDict('REWARD_DATA', msg.header)
        self.writeData('REWARD_DATA')



    #maybe synchronize laser and action callback
    def callback_laser(self, msg_action, msg_laser):
        self.data['LASER_DATA']['laser'] = [msg_laser.ranges[idx] for idx in self.a_selectScanTrack] 
        self.headerToDict('LASER_DATA', msg_laser.header)
        self.writeData('LASER_DATA')
#        rospy.Rate(1)



    def callback_sensors(self, msg):
        if(not self.b_SaveOtherData):
            self.data['SENSOR_DATA']['angle'] = msg.angle
            self.data['SENSOR_DATA']['trackpos'] = msg.trackPos
            self.data['SENSOR_DATA']['laptime'] = msg.currentLapTime
            self.data['SENSOR_DATA']['distance'] = msg.distFromStart
            self.headerToDict('SENSOR_DATA', msg.header)
            self.writeData('SENSOR_DATA')
            rospy.sleep(1)




    def callback_global(self, msg):
        if(not self.b_SaveOtherData):
            self.data['GLOBAL_DATA']['posX'] = msg.pose.position.x
            self.data['GLOBAL_DATA']['posY'] = msg.pose.position.y
            self.headerToDict('GLOBAL_DATA', msg.header)
            self.writeData('GLOBAL_DATA')
            rospy.sleep(0.2)




    def callback_speed(self, msg):
        if(not self.b_SaveOtherData):
            self.data['SPEED_DATA']['speedX'] = msg.twist.linear.x
            self.headerToDict('SPEED_DATA', msg.header)
            self.writeData('SPEED_DATA')
            rospy.sleep(1)


    def callback_ctrl(self, msg):
        if (msg.meta == 1 and self.write_ctrl == True):
            self.data['CTRL_DATA']['meta'] = msg.meta
            self.headerToDict('CTRL_DATA', msg.header)
            self.writeData('CTRL_DATA')
            self.write_ctrl = False
        else:
            self.write_ctrl = True

    def callback_startingPoint(self, msg):
        self.data['TRAJECTORY_DATA']['startingPoint'] = msg.data
        self.headerToDict('TRAJECTORY_DATA', msg.header)
        if (self.n_StartingPoint != msg.data):
            self.n_startingPoint = msg.data
            self.writeData('TRAJECTORY_DATA')

        
    def callback_dontSave(self, msg):
        self.b_SaveOtherData = msg.data

    def headerToDict(self, key1, header):
        self.data[key1]['ref_time']['secs'] = header.stamp.secs
        self.data[key1]['ref_time']['nsecs'] = header.stamp.nsecs

    def writeData(self, key1):
        hf = h5py.File(self.log_file, 'r+')
        for key2 in self.data[key1].keys():
            if(type(self.data[key1][key2]) == list):
                hf[key1][key2].resize((hf[key1][key2].shape[0]+1,len(self.data[key1][key2])))
            elif(type(self.data[key1][key2]) == dict):
                for key3 in self.data[key1][key2].keys():
                    hf[key1][key2][key3].resize((hf[key1][key2][key3].shape[0]+1,))
                    hf[key1][key2][key3][-1] = self.data[key1][key2][key3]
                continue #skip next assignement as key2 has been processed completely
            else:
                hf[key1][key2].resize((hf[key1][key2].shape[0]+1,))
            hf[key1][key2][-1] = self.data[key1][key2]

                
        hf.close()
        

        
    def createData(self):
        log_files = [f for f in os.listdir(self.save_directory) if ".h5" in f]
        if (len(log_files) == 0):
            self.log_file = self.save_directory + "/"+ self.save_directory[self.save_directory.rfind("/")+1:] + "_DATA01.h5"
        else:
            log_files.sort()
            self.log_file = log_files[-1]
            self.log_file = self.log_file[:-3]
            post = ""
            while(self.log_file[-1] >= '0' and self.log_file[-1] <= '9'):
                post = self.log_file[-1] + post
                self.log_file = self.log_file[:-1]
            post = int(post)+1
            pre = ''
            if (post <= 9):
                pre = '0'
            self.log_file = self.save_directory + "/" + self.log_file + pre + str(post) + ".h5"
        print(self.log_file)
        self.h5Groups = {}
        self.h5SubGroups = {}
        try:
          with h5py.File(self.log_file, 'w') as hf:
              for key1 in self.data.keys():
                  self.h5Groups[key1] = hf.create_group(key1)
                  for key2 in self.data[key1].keys():
                      if(type(self.data[key1][key2]) == list):
                          self.h5Groups[key1].create_dataset(key2, (0,len(self.data[key1][key2])), maxshape=(None,len(self.data[key1][key2])), 
                                       compression="gzip", compression_opts=9)
                      elif(type(self.data[key1][key2]) == dict):
                          self.h5SubGroups[key1] =  self.h5Groups[key1].create_group(key2)
                          for key3 in self.data[key1][key2].keys():
                              self.h5SubGroups[key1].create_dataset(key3, (0,), maxshape=(None,), compression="gzip", compression_opts=9, dtype='i')
                      else:
                          self.h5Groups[key1].create_dataset(key2, (0,), maxshape=(None,), compression="gzip", compression_opts=9)
              hf.close()
        except:
          print("Error occured in creation of save data")



if __name__ == '__main__':
    rospy.init_node('datalogger') #initalize code
    rospy.wait_for_message("/torcs_ros/notifications/InputReceived", Bool)
    [data_save, _, _, _, save_directory] = readConfigSrc(cwd)
    if(data_save):
        my_logger = DataLogger(cwd, save_directory) #create instance
        rospy.spin()