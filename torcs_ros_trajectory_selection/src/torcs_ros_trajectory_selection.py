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
import copy

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8
from torcs_msgs.msg import TORCSSensors, TORCSCtrl

import nengo_nets_qnet_associative as snn

import sys
import os
import rospkg

cwd = rospkg.RosPack().get_path('torcs_ros_trajectory_gen')
sys.path.append(cwd[:-24] + "common")
cwd = cwd[:-24]

from bzReadTrajectoryParams import readTrajectoryParams, calcTrajectoryAmount

#A class that can be passed to a nengo node as input
#It subscribes to the scan topic in order to create the nengo input
class NodeInputScan():
    def __init__(self, length, scan_topic = "/torcs_ros/scan_track"):
         #### various parameters and variables ####
        #choose index of scanners to use
        #angle min/max: +-1.57; increment 0.1653; instantenous; range: 200 m

        self.a_selectScanTrack = [3, 6, 11, -7, -6]
        self.param_rangeNormalize = 100 #value used for normalization. all values above will be considered as 1   
  
        #### subscription parameters ####      
        self.a_scanTrack = []
#        [self.a_scanTrack.append(-1) for idx in self.a_selectScanTrack]
        [self.a_scanTrack.append(-1) for idx in range(length)]

        
        ##### subscribers #####
        self.sub_scanTrack = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        
    #Get values from message, normalize them and clip them between 0 and 1
    def scan_callback(self, msg_scan):
        pass
#        self.a_scanTrack = [np.clip(msg_scan.ranges[idx]/self.param_rangeNormalize, 0, 1) for idx in self.a_selectScanTrack]
        
    #Call function needed for nengo, called within each simulation step 
    #returns a constant input over the entirety of the simulation dependent on the sensor state when starting the simulation
    def __call__(self, t):
        return self.a_scanTrack
    
    #Used for debug purposes to ensure object has not been destructed
    def __del__(self):
        print("The msg to nengo object is being destructed")
        
    #Needed for nengo net to be parametrizable
    def __len__(self):
        return len(self.a_selectScanTrack)
    
    def setVals(self, ia_scans):
        self.a_scanTrack = ia_scans
        
        
#A class that can be passed to a nengo node as input
#Gets the reward vector from ros whenever a trajectory has been completed
#Returns that reward vector on every timestep with the __call__ function        
class NodeInputReward():
    def __init__(self, n_action):
        self.reward = n_action*[0] #initialize reward to be 0 for all actions
        
    def __call__(self, t):
        return self.reward #return current reward
    
    def NoLearning(self):
        self.reward = [0 for action in self.reward] #reset reward array to 0
        
    def RewardAction(self, i, i_reward):
        self.reward[i] = i_reward #set a specific action to have a reward

#A class that can be passed to a nengo node as an output probe
#Member variables only save last state, therefore eliminating a need for a nengo.Probe or similar
class NodeOutputProber():
    def __init__ (self, n_action):
        self.probe_vals = [0 for n in range(n_action+1)] #init array, one entry each for one-hot encoding and one for the highest q-value
        self.time_val = 0 #can save current simulation time as well, not used currently
    def ProbeFunc(self, t, x):
        self.probe_vals = x #update members to nengo output
        self.time_val = t 
        return self.probe_vals

        
class TrajectorySelector():
    def __init__(self, cwd, scan_topic = "/torcs_ros/scan_track", action_topic="/torcs_ros/ctrl_signal_action",
                 sensors_topic = "/torcs_ros/sensors_state", speed_topic="/torcs_ros/speed", 
                 ctrl_topic = "torcs_ros/ctrl_cmd"):
  
        #### various parameters and variables ####
        #choose index of scanners to use
        #angle min/max: +-1.57; increment 0.1653; instantenous, range: 200 m
        self.a_selectScanTrack = [3, 6, 11, -7, -6]
        self.param_rangeNormalize = 100 #value used for normalization. all values above will be considered as 1
        self.param_f_maxExpectedSpeed = 37.0 #[km/h], set higher than 34 deliberately to scale reward a bit
        [self.param_f_longitudinalDist, _, self.param_n_action] = readTrajectoryParams(cwd)
        self.param_n_action = calcTrajectoryAmount(self.param_n_action)
        self.calculateRewardRange()
        
        #### subscription parameters ####
        self.b_TrajectoryNeeded = False
        self.f_lapTimeStart = 0
        self.f_lapTimeCurrent = 0 
        self.f_lapTimePrevious = 0#needed for lap change
        self.f_distStart = 0
        self.f_distCurrent = 0
        self.f_distPrevious = 0
        self.f_speedXCurrent = 0
        self.f_speedXStart = 0
        self.a_scanTrack = []
        [self.a_scanTrack.append(-1) for idx in self.a_selectScanTrack]
        self.b_handshake = False

        
        #### nengo net and parameters #### 
        self.state_inputer = NodeInputScan(len(self.a_selectScanTrack))
        self.reward_inputer = NodeInputReward(self.param_n_action)
        self.output_prober = NodeOutputProber(self.param_n_action) #naction currently hardocded, should be a global ros parameter
        self.q_net_ass = snn.qnet_associative(False, self.state_inputer, self.reward_inputer, 
                                              self.output_prober.ProbeFunc, self.param_n_action)
        nengo.rc.set('progress', 'progress_bar', 'nengo.utils.progress.TerminalProgressBar') #Terminal progress bar for inline
        self.sim = nengo.Simulator(self.q_net_ass, progress_bar=False)
        self.b_doSimulateOnce = True
        self.idx_action = 0
        
        ##### subscribers #####
        self.sub_scanTrack = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        self.sub_needForAction = rospy.Subscriber(action_topic, Bool, self.needForAction_callback)
        self.sub_sensors = rospy.Subscriber(sensors_topic, TORCSSensors, self.sensors_callback)
        self.sub_speed = rospy.Subscriber(speed_topic, TwistStamped, self.speed_callback)
        self.sub_handshake = rospy.Subscriber("/torcs_ros/gen2selHandshake", Bool, self.handshake_callback)
        self.sub_ctrlCmd = rospy.Subscriber(ctrl_topic, TORCSCtrl, self.ctrl_callback)
        
        #### publisher ####
        self.pub_trajectorySelection = rospy.Publisher("/torcs_ros/TrajectorySelector", Int8, queue_size=1) #negative values are parsed when no trajectory should be selected
        
        
    def scan_callback(self, msg_scan):
        self.a_scanTrack = [np.clip(msg_scan.ranges[idx]/self.param_rangeNormalize, 0, 1) for idx in self.a_selectScanTrack]

    def needForAction_callback(self, msg_action):
        self.b_TrajectoryNeeded = msg_action.data
        if (self.b_TrajectoryNeeded == True):
            if self.b_doSimulateOnce:
                self.calculateReward()
                ###select previously run output and resimulate in off-time to calculate next output
                msg_sel = Int8()
                msg_sel.data = self.idx_action
                self.pub_trajectorySelection.publish(msg_sel)
                self.f_lapTimeStart = self.f_lapTimeCurrent
                self.f_distStart = self.f_distCurrent
                self.f_speedXStart = self.f_speedXCurrent
                
                #enable faster velocities by pausing game and allowing for simulation
                #not very nice but might be needed with bad hardware
                
                #needs xdotool
                #os.system('xdotool search --name "torcs-bin" key p')
                #subprocess.call('xdotool search --name "torcs-bin" key p', shell=True) 
                self.state_inputer.setVals(self.a_scanTrack) #state values only changed before action selection simulation
                self.sim.run(0.5) #run simulation for x 
                self.idx_action = np.argmax(np.array(self.output_prober.probe_vals[:-1]))
                self.b_doSimulateOnce = False #don't simualte again until this trajectory has been published (which means that the action signal will turn to False)
                
        else:
            self.b_doSimulateOnce = True #action signal is not set anymore, on next true we need to perform another simulation
        if (self.b_handshake == False):
            msg_sel = Int8()
            msg_sel.data = self.idx_action
            self.pub_trajectorySelection.publish(msg_sel)
            ##reset do once flag, a new trajectory has been published

    def sensors_callback(self, msg_sensors):
        self.f_lapTimePrevious = self.f_lapTimeCurrent
        self.f_lapTimeCurrent = msg_sensors.currentLapTime
        
        self.f_distPrevious = self.f_distCurrent
        self.f_distCurrent = msg_sensors.distFromStart
        
        if(self.f_lapTimeCurrent < self.f_lapTimeStart): 
            self.f_lapTimeStart = -(self.f_lapTimePrevious - self.f_lapTimeStart)
        if(self.f_distCurrent*10 < self.f_distStart): #*10 ensures condition to only hold at lap change
            self.f_distStart = -(self.f_distPrevious - self.f_distStart)
            print("Crossing start line")
            
        
    def speed_callback(self, msg_speed):
        self.f_speedXCurrent = msg_speed.twist.linear.x
        
    def handshake_callback(self, msg_handshake):
        self.b_handshake = msg_handshake.data
        
    def ctrl_callback(self, msg_ctrl):
        self.b_handshake = True
        if(msg_ctrl.meta == 1):
            print("A restart has been requested")
            self.clearMemory()
        
        
    #calculate the lowest amount of time needed at the expected speed to traverse the longitudinal distance if the road were to be straight
    #this will then be used to calculate the reward
    #higher values can be achieved in curves, but it is not absoulutely necessary to limit this value to one
    def calculateRewardRange(self):
        self.param_f_minTime = self.param_f_longitudinalDist / (self.param_f_maxExpectedSpeed*1000.0/3600)
        
        
        
    def calculateReward(self):
        if (self.f_speedXCurrent < 30 or self.f_speedXStart < 30): #disregard when end speed has not been reached yet
            self.reward = 0
        elif (self.f_lapTimeCurrent > self.f_lapTimeStart): #ensure no new lap has started
            f_distTravelled = self.f_distCurrent - self.f_distStart #
            f_timeNeeded = self.f_lapTimeCurrent - self.f_lapTimeStart #can be neglected if we 
            self.reward = (f_distTravelled/self.param_f_longitudinalDist) / (f_timeNeeded/self.param_f_minTime) #maybe pow 2
            self.reward *= self.reward #scale reward for more distinction between all trajectories
        self.checkRewardValidity()
        
        print(self.reward)

        
    def checkRewardValidity(self):
        if (self.reward > 5):
            self.reward = 0
            
    def clearMemory(self):
        pass

if __name__ == "__main__":
    if(len(os.popen("dpkg -l | grep xdotool").readlines()) == 0): #check whether xdotool is installed
        print("\033[93mIt seems like xdotool is not installed! Please install it as torcs_ros_trajectory_selection relies on it \033[97m")
    rospy.init_node("trajectory_selection")
    selector = TrajectorySelector(cwd)
    rospy.spin()
    