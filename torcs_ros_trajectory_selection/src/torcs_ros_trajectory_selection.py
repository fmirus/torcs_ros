#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 26 10:23:30 2018

@author: bzorn
"""

import numpy as np
import rospy
import nengo
import subprocess

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8
from torcs_msgs.msg import TORCSSensors

import nengo_nets_qnet_associative as snn

class NodeInputScan():
    def __init__(self, scan_topic = "/torcs_ros/scan_track"):
         #### various parameters and variables ####
        #choose index of scanners to use
        #angle min/max: +-1.57; increment 0.1653; instantenous, range: 200 m
        self.a_selectScanTrack = [3, 6, 11, -7, -6]
        self.param_rangeNormalize = 100 #value used for normalization. all values above will be considered as 1   
  
        #### subscription parameters ####      
        self.a_scanTrack = []
        [self.a_scanTrack.append(0) for idx in self.a_selectScanTrack]
        
        ##### subscribers #####
        self.sub_scanTrack = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        
    def scan_callback(self, msg_scan):
        self.a_scanTrack = [np.clip(msg_scan.ranges[idx]/self.param_rangeNormalize, 0, 1) for idx in self.a_selectScanTrack]
        
    def __call__(self, t):
        return self.a_scanTrack
    def __del__(self):
        print("The msg to nengo object is being destructed")
        
    def __len__(self):
        return len(self.a_selectScanTrack)

class NodeOutputProber():
    def __init__ (self, n_action):
        self.probe_vals = [0 for n in range(n_action+1)]
        self.time_val = 0
    def ProbeFunc(self, t, x):
        self.probe_vals = x # array as could have several dimensions
        self.time_val = t
        return self.probe_vals
#    def __call__(self, t):
#        
#        return self.probe_vals
        
class TrajectorySelector():
    def __init__(self, scan_topic = "/torcs_ros/scan_track", action_topic="/torcs_ros/ctrl_signal_action",
                 sensors_topic = "/torcs_ros/sensors_state", speed_topic="/torcs_ros/speed"):
  
        #### various parameters and variables ####
        #choose index of scanners to use
        #angle min/max: +-1.57; increment 0.1653; instantenous, range: 200 m
        self.a_selectScanTrack = [3, 6, 11, -7, -6]
        self.param_rangeNormalize = 100 #value used for normalization. all values above will be considered as 1
        
        #### subscription parameters ####
        self.b_TrajectoryNeeded = False
        self.f_lapTimeStart = 0
        self.f_lapTimeEnd = 0 #not really needed
        self.f_lapTimeCurrent = 0 
        self.f_distStart = 0
        self.f_distEnd = 0 #not really needed
        self.f_distCurrent = 0
        self.f_speedX = 0
        self.a_scanTrack = []
        [self.a_scanTrack.append(0) for idx in self.a_selectScanTrack]
        
        
        #### nengo net and parameters #### 
        self.output_prober = NodeOutputProber(21) #naction currently hardocded, should be a global ros parameter
        self.q_net_ass = snn.qnet_associative(True, NodeInputScan(), self.output_prober.ProbeFunc)
        nengo.rc.set('progress', 'progress_bar', 'nengo.utils.progress.TerminalProgressBar') #Terminal progress bar for inline
        self.sim = nengo.Simulator(self.q_net_ass, progress_bar=True)
        self.b_doSimulateOnce = True
        self.idx_action = 0
        ##### subscribers #####
        self.sub_scanTrack = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        self.sub_needForAction = rospy.Subscriber(action_topic, Bool, self.needForAction_callback)
        self.sub_sensors = rospy.Subscriber(sensors_topic, TORCSSensors, self.sensors_callback)
        self.sub_speed = rospy.Subscriber(speed_topic, TwistStamped, self.speed_callback)
        
        #### publisher ####
        self.pub_trajectorySelection = rospy.Publisher("/torcs_ros/TrajectorySelector", Int8, queue_size=1) #negative values are parsed when no trajectory should be selected
        
        
    def scan_callback(self, msg_scan):
        self.a_scanTrack = [np.clip(msg_scan.ranges[idx]/self.param_rangeNormalize, 0, 1) for idx in self.a_selectScanTrack]
#        print(self.a_scanTrack)

    def needForAction_callback(self, msg_action):
        self.b_TrajectoryNeeded = msg_action.data
        if (self.b_TrajectoryNeeded == True):
            if self.b_doSimulateOnce:
                ###select previously run output and resimulate in off-time to calculate next output
                msg_action = Int8()
                msg_action.data = self.idx_action
                self.pub_trajectorySelection.publish(msg_action)
                self.f_lapTimeStart = self.f_lapTimeCurrent
                self.f_distStart = self.f_distCurrent
                self.sim.run(0.5) #run simulation for x 
                self.idx_action = np.argmax(np.array(self.output_prober.probe_vals[:-1]))
                self.b_doSimulateOnce = False #don't simualte again until this trajectory has been published (which means that the action signal will turn to False)

        else:
            self.b_doSimulateOnce = True #action signal is not set anymore, on next true we need to perform another simulation
            
            ##reset do once flag, a new trajectory has been published

    def sensors_callback(self, msg_sensors):
        self.f_lapTimeCurrent = msg_sensors.currentLapTime
        self.f_distCurrent = msg_sensors.distFromStart
        
    def speed_callback(self, msg_speed):
        self.f_speedX = msg_speed.twist.linear.x


if __name__ == "__main__":
    rospy.init_node("trajectory_selection")
    selector = TrajectorySelector()
    rospy.spin()
    