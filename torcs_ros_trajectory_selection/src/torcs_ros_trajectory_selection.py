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
    def __init__(self, cwd, scan_topic = "/torcs_ros/scan_track", action_topic="/torcs_ros/notifications/ctrl_signal_action",
                 sensors_topic = "/torcs_ros/sensors_state", speed_topic="/torcs_ros/speed", 
                 ctrl_topic = "torcs_ros/ctrl_cmd"):
  
        #### various parameters and variables ####
        #choose index of scanners to use
        #angle min/max: +-1.57; increment 0.1653; instantenous, range: 200 m
        self.a_selectScanTrack = [3, 6, 11, -7, -4]
        self.param_rangeNormalize = 200 #value used for normalization. all values above will be considered as 1
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
        self.state_inputer = NodeInputScan(len(self.a_selectScanTrack)) #object passed to nengo net with scan values as input
        self.reward_inputer = NodeInputReward(self.param_n_action) #object passed to nengo net with reward as input
        self.time_inputer = NodeInputStartTime() #object passed to nengo net with current simulations starting time as input
        self.epsilon_inputer = NodeInputEpsilon(self.param_n_action) #object passed to nengo implementing epsilon greedy exploration
        self.output_prober = NodeOutputProber(self.param_n_action) #naction currently hardocded, should be a global ros parameter
        self.q_net_ass = snn.qnet_associative(False, self.state_inputer, self.reward_inputer, self.time_inputer,
                                              self.epsilon_inputer, self.output_prober.ProbeFunc, self.param_n_action) #construct nengo net with proper inputs
        nengo.rc.set('progress', 'progress_bar', 'nengo.utils.progress.TerminalProgressBar') #Terminal progress bar for inline
#        self.sim = nengo.Simulator(self.q_net_ass, progress_bar=True, optimize=True) #optimize trades in build for simulation time
        self.sim = nengo_dl.Simulator(self.q_net_ass, progress_bar=False) #use nengo_dl simulator for reduced simulation time and parameter saving feature
        
        self.b_doSimulateOnce = True #flag used to prevent repeated/unneeded simulations
        self.idx_last_action = 0 #index of trajectory/action used last, saved for training
        self.idx_next_action = 0 #index of trajectory/action to be used next

        #### publisher ####
        self.msg_pause = Bool() #message used to demand gamestate node to pause game
        self.msg_nengo = Bool() #message used to notify other nodes that a nengo calculation is going on and the game should not be unpaused meanwhile

        self.pub_trajectorySelection = rospy.Publisher("/torcs_ros/TrajectorySelector", Int8, queue_size=1) #negative values are parsed when no trajectory should be selected
        self.pub_demandPause = rospy.Publisher("/torcs_ros/notifications/demandPause", Bool,queue_size = 1) #a pause request is sent with every publish, independent of data
        self.pub_nengoRunning = rospy.Publisher("/torcs_ros/notifications/nengoIsRunning", Bool, queue_size=1) #last published value is current calculation status (true: calculation running, false:no calculation)
        
        ##### subscribers #####
        self.sub_scanTrack = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback) #get laser scanners
        self.sub_needForAction = rospy.Subscriber(action_topic, Bool, self.needForAction_callback) #get notification that new action idx is needed 
        self.sub_sensors = rospy.Subscriber(sensors_topic, TORCSSensors, self.sensors_callback) #get sensor values
        self.sub_speed = rospy.Subscriber(speed_topic, TwistStamped, self.speed_callback) #get speed values
        self.sub_handshake = rospy.Subscriber("/torcs_ros/gen2selHandshake", Bool, self.handshake_callback) #//depreceated; ensures that generation and selection nodes are both up
        self.sub_ctrlCmd = rospy.Subscriber(ctrl_topic, TORCSCtrl, self.ctrl_callback) #check for meta command
        self.sub_restart = rospy.Subscriber("/torcs_ros/notifications/restart_process", Bool, self.restart_callback) #see whether client is restarting torcs
        self.sub_save = rospy.Subscriber("/torcs_ros/notifications/save", Bool, self.save_callback) #check for manual save nengo command
        
        
        
    def scan_callback(self, msg_scan):
        self.a_scanTrack = [np.clip(msg_scan.ranges[idx]/self.param_rangeNormalize, 0, 1) for idx in self.a_selectScanTrack] #normalize values and clip them additionally (clip/normalization has pending change)

    #Checks whether a new trajector is needed
    def needForAction_callback(self, msg_action):
        self.b_TrajectoryNeeded = msg_action.data
        if (self.b_TrajectoryNeeded == True): #if a new trajectory is needed
            if self.b_doSimulateOnce: #ensures simulation is only performed once
                self.pub_demandPause.publish(self.msg_pause) #demand pause in order to perform simulation
                self.msg_nengo.data = True
                self.pub_nengoRunning.publish(self.msg_nengo.data) #inform that a calculation is in progress and game should remain paused

                self.calculateReward() #calculate the last action's reward
                
                ###select previously run output and resimulate in off-time to calculate next output
                msg_sel = Int8()
                msg_sel.data = self.idx_next_action
#                msg_sel.data = 0 #used in debug cases
                self.pub_trajectorySelection.publish(msg_sel) #publish trajectory calculated in the last step
                #set new actions start values used for reward calculation in next step
                self.f_lapTimeStart = self.f_lapTimeCurrent
                self.f_distStart = self.f_distCurrent
                self.f_speedXStart = self.f_speedXCurrent

                self.state_inputer.setVals(self.a_scanTrack) #state values only changed before action selection simulation
                self.epsilon_inputer.Explore() #prepare for action selection, performs an epsilon greedy algorithm
                self.epsilon_inputer.SetActive() #ensure that no training is to happen
                self.time_inputer.setT(self.output_prober.time_val) #input simulation starting time to node
                self.sim.run(0.5, progress_bar = False) #run simulation for x 
                self.pub_demandPause.publish(self.msg_pause) #demand game unpause
                self.msg_nengo.data = False
                self.pub_nengoRunning.publish(self.msg_nengo.data) #notify that nengo is not calculating anymore
                self.idx_last_action = self.idx_next_action #save last used action for reward training
                self.idx_next_action = np.argmax(np.array(self.output_prober.probe_vals[:-1])) #get next action from output
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
        #Check whether the start line is being crossed and adjust start time and distances to handle such a scenaro
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

    #Train with negative reward on restart, as car is either stuck or off track due to last performed action          
    def restart_callback(self, msg_restart):
        if (msg_restart.data == True): #if restart occurs
            if(self.b_hasBeenTrained == False): #this flag ensures that the training is performed only once per restart
                self.b_hasBeenTrained = True 
#                print("A restart has been requested") 
                self.reward = -5 #negative reward value
                self.pub_demandPause.publish(self.msg_pause) #demand game pause
                self.msg_nengo.data = True
                self.pub_nengoRunning.publish(self.msg_nengo.data) #notify that nengo is calculating
                self.trainOnReward() #train network with negative reward
                self.msg_nengo.data = False
                self.pub_nengoRunning.publish(self.msg_nengo.data) #notify that nengo is not calculating anymore
                self.pub_demandPause.publish(self.msg_pause) #demand game unpause
        else:
            self.b_hasBeenTrained = False #game has been restart, flag can be reset therefore
        
    def save_callback(self, msg_save):
        self.saveNengoParams() #saves nengo parameters when something is published to this topic
        
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
            self.reward += (1-abs(self.f_trackPos)) # + (1-abs(self.f_angle)/2) or delta trackpos and delta angle (only in associative) 
        self.checkRewardValidity() #ensure no weird rewards have been calculated
        self.trainOnReward() #traing
        

        
    def checkRewardValidity(self):
        if (self.reward > 5):
            self.reward = np.nan
            
    def clearMemory(self):
        pass
    
    def trainOnReward(self):
        if not(np.isnan(self.reward)): #no need to run if the reward does not count
            print("Training action \033[96m" + str(self.idx_last_action) + "\033[0m with reward: \033[96m" +str(self.reward) + "\033[0m") #console notifcation
            self.reward_inputer.RewardAction(self.idx_last_action, self.reward) #input reward to nengo node
            self.epsilon_inputer.OnTraining() #prepare for training
            self.epsilon_inputer.SetTraining() #update epsilon values
            self.sim.run(0.5,  progress_bar = False) #train
            if((self.epsilon_inputer.episode-2) % 200 == 0): #save intermediate nengo parameters every x parameters
                self.saveNengoParams()
 
            self.reward_inputer.NoLearning() #ensure no training is performed before next trainOnReward call

    #saves nengo parameters with nengo_dl feature with epsiode number and current date as name
    def saveNengoParams(self):
        dir_name = self.cwd[:-14] + "nengo_parameters"
        if not os.path.isdir(dir_name):
            os.mkdir(dir_name)
        path_name = dir_name + "/Episode-"+ str(int(self.epsilon_inputer.episode)-2)+"_Date-" + str(self.today.year) + "-" + str(self.today.month) + "-" + str(self.today.day)
        print("\033[96mEpisode " + str(int(self.epsilon_inputer.episode-2)) + " reached. Saving parameters to " + path_name + "\033[0m")
        self.sim.save_params(path_name)
        

if __name__ == "__main__":
    rospy.init_node("trajectory_selection")
    selector = TrajectorySelector(cwd)
    rospy.spin()
    