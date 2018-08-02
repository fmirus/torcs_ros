#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 26 10:23:30 2018

@author: bzorn
"""

import numpy as np
import rospy
import nengo
import nengo_dl
import subprocess


from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8
from torcs_msgs.msg import TORCSSensors, TORCSCtrl

import nengo_nets_qnet_associative as snn

import sys
import os
import rospkg
import datetime

cwd = rospkg.RosPack().get_path('torcs_ros_trajectory_gen')
sys.path.append(cwd[:-24] + "common")
cwd = cwd[:-24]

from bzReadTrajectoryParams import readTrajectoryParams, calcTrajectoryAmount
from ros_to_nengo_nodes import NodeInputScan, NodeInputReward, NodeInputStartTime, NodeInputEpsilon, NodeOutputProber


        
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
        self.param_n_action = calcTrajectoryAmount(self.param_n_action) #get how many trajectories are actually used
        self.calculateRewardRange() #calculate normalization factor for reward from parameters
        self.reward = np.nan
        self.cwd = cwd
        self.today = datetime.date.today()
        
        #### subscription parameters ####
        self.f_angle = 0
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
        self.f_trackPos = 0
        self.b_hasBeenTrained = False

        
        #### nengo net and parameters #### 
        self.state_inputer = NodeInputScan(len(self.a_selectScanTrack))
        self.reward_inputer = NodeInputReward(self.param_n_action)
        self.time_inputer = NodeInputStartTime()
        self.epsilon_inputer = NodeInputEpsilon(self.param_n_action)
        self.output_prober = NodeOutputProber(self.param_n_action) #naction currently hardocded, should be a global ros parameter
        self.q_net_ass = snn.qnet_associative(False, self.state_inputer, self.reward_inputer, self.time_inputer,
                                              self.epsilon_inputer, self.output_prober.ProbeFunc, self.param_n_action)
        nengo.rc.set('progress', 'progress_bar', 'nengo.utils.progress.TerminalProgressBar') #Terminal progress bar for inline
#        self.sim = nengo.Simulator(self.q_net_ass, progress_bar=True, optimize=True) #optimize trades in build for simulation time
        self.sim = nengo_dl.Simulator(self.q_net_ass, progress_bar=False)
        
        self.b_doSimulateOnce = True
        self.idx_last_action = 0
        self.idx_next_action = 0
        self.msg_pause = Bool()

        #### publisher ####
        self.pub_trajectorySelection = rospy.Publisher("/torcs_ros/TrajectorySelector", Int8, queue_size=1) #negative values are parsed when no trajectory should be selected
        self.pub_demandPause = rospy.Publisher("/torcs_ros/demandPause", Bool,queue_size = 1)
        
        
        ##### subscribers #####
        self.sub_scanTrack = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        self.sub_needForAction = rospy.Subscriber(action_topic, Bool, self.needForAction_callback)
        self.sub_sensors = rospy.Subscriber(sensors_topic, TORCSSensors, self.sensors_callback)
        self.sub_speed = rospy.Subscriber(speed_topic, TwistStamped, self.speed_callback)
        self.sub_handshake = rospy.Subscriber("/torcs_ros/gen2selHandshake", Bool, self.handshake_callback)
        self.sub_ctrlCmd = rospy.Subscriber(ctrl_topic, TORCSCtrl, self.ctrl_callback)
        self.sub_restart = rospy.Subscriber("/torcs_ros/restart_process", Bool, self.restart_callback)

    def scan_callback(self, msg_scan):
        self.a_scanTrack = [np.clip(msg_scan.ranges[idx]/self.param_rangeNormalize, 0, 1) for idx in self.a_selectScanTrack]

    def needForAction_callback(self, msg_action):
        self.b_TrajectoryNeeded = msg_action.data
        if (self.b_TrajectoryNeeded == True):
            if self.b_doSimulateOnce:
                self.pub_demandPause.publish(self.msg_pause)
                self.calculateReward()
                ###select previously run output and resimulate in off-time to calculate next output
                msg_sel = Int8()
                msg_sel.data = self.idx_next_action
#                msg_sel.data = 0
                self.pub_trajectorySelection.publish(msg_sel)
                self.f_lapTimeStart = self.f_lapTimeCurrent
                self.f_distStart = self.f_distCurrent
                self.f_speedXStart = self.f_speedXCurrent
                
                #enable faster velocities by pausing game and allowing for simulation
                #not very nice but might be needed with bad hardware
                
                #needs xdotool
 
                self.state_inputer.setVals(self.a_scanTrack) #state values only changed before action selection simulation
                self.epsilon_inputer.Explore() #prepare for action selection
                self.epsilon_inputer.SetActive()
                self.time_inputer.setT(self.output_prober.time_val)
                self.sim.run(0.5, progress_bar = False) #run simulation for x 
                self.pub_demandPause.publish(self.msg_pause)
                self.idx_last_action = self.idx_next_action
                self.idx_next_action = np.argmax(np.array(self.output_prober.probe_vals[:-1]))
                self.b_doSimulateOnce = False #don't simualte again until this trajectory has been published (which means that the action signal will turn to False)
                
        else:
            self.b_doSimulateOnce = True #action signal is not set anymore, on next true we need to perform another simulation
        if (self.b_handshake == False):
            msg_sel = Int8()
            msg_sel.data = self.idx_next_action
            self.pub_trajectorySelection.publish(msg_sel)
            ##reset do once flag, a new trajectory has been published

    def sensors_callback(self, msg_sensors):
        self.f_lapTimePrevious = self.f_lapTimeCurrent
        self.f_lapTimeCurrent = msg_sensors.currentLapTime
        
        self.f_distPrevious = self.f_distCurrent
        self.f_distCurrent = msg_sensors.distFromStart
        self.f_trackPos = msg_sensors.trackPos
        self.f_angle = msg_sensors.angle
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
#        if(msg_ctrl.meta == 1):
#            print("A restart has been requested")
#            self.reward = -5
#            self.pub_demandPause.publish(self.msg_pause)
#            self.trainOnReward()
#            self.pub_demandPause.publish(self.msg_pause)
          
    def restart_callback(self, msg_restart):
        if (msg_restart.data == True):
            if(self.b_hasBeenTrained == False):
                self.b_hasBeenTrained = True
                print("A restart has been requested")
                self.reward = -5
                self.pub_demandPause.publish(self.msg_pause)
                self.trainOnReward()
                self.pub_demandPause.publish(self.msg_pause)
        else:
            self.b_hasBeenTrained = False
        
    #calculate the lowest amount of time needed at the expected speed to traverse the longitudinal distance if the road were to be straight
    #this will then be used to calculate the reward
    #higher values can be achieved in curves, but it is not absoulutely necessary to limit this value to one
    def calculateRewardRange(self):
        self.param_f_minTime = self.param_f_longitudinalDist / (self.param_f_maxExpectedSpeed*1000.0/3600)
        
        
        
    def calculateReward(self):
        if (self.f_speedXCurrent < 30 or self.f_speedXStart < 30): #disregard when end speed has not been reached yet
            self.reward = np.nan
        elif (self.f_lapTimeCurrent > self.f_lapTimeStart): #ensure no new lap has started
            f_distTravelled = self.f_distCurrent - self.f_distStart #
            f_timeNeeded = self.f_lapTimeCurrent - self.f_lapTimeStart #can be neglected if we 
            self.reward = (f_distTravelled/self.param_f_longitudinalDist) / (f_timeNeeded/self.param_f_minTime) #maybe pow 2
            self.reward *= self.reward #scale reward for more distinction between all trajectories
            self.reward += (1-abs(self.f_trackPos)) # + (1-abs(self.f_angle)/2)
        self.checkRewardValidity()
        self.trainOnReward()
        

        
    def checkRewardValidity(self):
        if (self.reward > 5):
            self.reward = np.nan
            
    def clearMemory(self):
        pass
    
    def trainOnReward(self):
        if not(np.isnan(self.reward)): #no need to run if the reward does not count
            print("Training action \033[96m" + str(self.idx_last_action) + "\033[0m with reward: \033[96m" +str(self.reward) + "\033[0m")
            self.reward_inputer.RewardAction(self.idx_last_action, self.reward)
            self.epsilon_inputer.OnTraining()
            self.epsilon_inputer.SetTraining()
            self.sim.run(0.5,  progress_bar = False)
            if((self.epsilon_inputer.episode-2) % 200 == 0):
                dir_name = self.cwd[:-14] + "nengo_parameters"
                if not os.path.isdir(dir_name):
                    os.mkdir(dir_name)
                path_name = dir_name + "/Episode-"+ str(int(self.epsilon_inputer.episode)-2)+"_Date-" + str(self.today.year) + "-" + str(self.today.month) + "-" + str(self.today.day)
                    
                print("\033[96mEpisode " + str(int(self.epsilon_inputer.episode-2)) + " reached. Saving parameters to " + path_name + "\033[0m")
                self.sim.save_params(path_name)
            self.reward_inputer.NoLearning()



if __name__ == "__main__":
    rospy.init_node("trajectory_selection")
    selector = TrajectorySelector(cwd)
    rospy.spin()
    