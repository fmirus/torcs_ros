#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Aug  1 08:35:20 2018

@author: bzorn
"""
import copy
import numpy as np


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
    
    #Manually set values as constant node input (has to be called before every simulation.run call)
    def setVals(self, ia_scans):
        self.a_scanTrack = ia_scans
        
 
#A class that can be passed to a nengo node as input
#Gets the reward vector from ros whenever a trajectory has been completed
#Returns that reward vector on every timestep with the __call__ function        
class NodeInputReward():
    def __init__(self, n_action):
        self.reward = 0 #initialize reward to be 0 for all actions
        self.oneHot_action = n_action*[1] #inverse one hot definining which action to teach, as it is used as inhibiting connection for learning
        self.retVec = (n_action+1)*[1] #return vector that includes inverse one hot as well as reward as last value
        
    #called on every timestep of simulation
    def __call__(self, t):
        return self.retVec #return current reward
    
    #call if learning should be supressed (should be called before action selection)
    def NoLearning(self):
        self.reward = 0 #reset reward array to 0
        self.oneHot_action = [1 for f in self.oneHot_action] #ensure all actions are inhibited
        #build return vector
        self.retVec = copy.copy(self.oneHot_action)
        self.retVec.append(self.reward) 
        
    def RewardAction(self, i, i_reward):
        self.reward = i_reward #set a specific action to have a reward
        self.oneHot_action[i] = 0 #learn action i by not inhibiting it
        #build return vector
        self.retVec = copy.copy(self.oneHot_action)
        self.retVec.append(self.reward) 
        self.oneHot_action[i] = 1 #reset to ensure no two actions are rewarded if NoLearning wasn't called in between
        
        
class NodeInputStartTime():
    def __init__(self):
        self.t_start = 0
    def __call__(self, t):
        return self.t_start
    def setT(self, t):
        self.t_start = t
    
class NodeInputEpsilon():
    def __init__(self, in_action):
        self.epsilon_init = 0.5
        self.epsilon = copy.copy(self.epsilon_init)
        self.val = -1
        self.lastVal = -1
        self.nextVal = -1
        self.n_action = in_action
        self.episode = 1.0
        
    def DoNotExplore(self):
        self.val = -1
    def Explore(self):
        rand = np.random.uniform(0, 1)
        self.lastVal = self.nextVal
        if (rand < self.epsilon):
            self.nextVal = np.random.randint(0, self.n_action-1)
        else:
            self.nextVal = -1
        
        if (self.lastVal != -1):
            print("Exploring action: " +str(self.lastVal) + " with current th_epsilon: " +str(self.epsilon))
            
    def SetActive(self):
        self.val = self.nextVal
    def SetTraining(self):
        self.val = self.lastVal
    def __call__(self, t):
        return self.val
    def OnTraining(self):
        self.episode += 1
#        self.epsiolon_init
        self.epsilon = np.clip(self.epsilon_init/(float(self.episode)/150), 0, 1)

        

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
    
class NodeInhibitAlLTraining():
    def __init__ (self):
        self.b_DoInhibit = False
    def __call__(self, t):
        if (self.b_DoInhibit == True):
            return 1
        else:
            return 0
    def InhibitTrainingSubnetwork(self):
        self.b_DoInhibit = True
    def DoNotInhibitTrainingSubnetwork(self):
        self.b_DoInhibit = False
    