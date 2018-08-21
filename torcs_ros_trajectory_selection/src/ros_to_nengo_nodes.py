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
    def __init__(self, i_aScantrack,  scan_topic = "/torcs_ros/scan_track"):
         #### various parameters and variables ####
        #choose index of scanners to use
        #angle min/max: +-1.57; increment 0.1653; instantenous; range: 200 m

        self.a_selectScanTrack = i_aScantrack
        self.param_rangeNormalize = 150 #value used for normalization. all values above will be considered as 1   
  
        #### subscription parameters ####      
        self.a_scanTrack = []
#        [self.a_scanTrack.append(-1) for idx in self.a_selectScanTrack]
        [self.a_scanTrack.append(-1) for idx in range(len(self.a_selectScanTrack))]

        
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
        
#A class that can be passed to a nengo node as input
#Returns the (externally set) start time
#This node is used to limit the learning to a certain window of time after the current simulation has started         
class NodeInputStartTime():
    def __init__(self):
        self.t_start = 0 #start time of current simulation.run
    def __call__(self, t):
        return self.t_start #always return start time
    def setT(self, t):
        self.t_start = t #set start time (to be called before running a simulation)

#A class that can be passed to a nengo node as input
#Implements a decaying epsilon greedy exploration. Returns the index of the chosen action if exploration is desired
#Nengo net is configured to use deterministic argmax if the returned idx value is -1
#Otherwise the epsilon exploration of the returned idx is used
class NodeInputEpsilon():
    def __init__(self, in_action, i_epsilon_init=0.5, i_decay=250):
        self.epsilon_init = i_epsilon_init #initial epsilon value
        self.epsilon = copy.copy(self.epsilon_init) #current epsilon value
        self.epsilon_temp = 0
        self.val = -1 #return action
        self.lastVal = -1 #action idx used previously, needed for training
        self.nextVal = -1 #action idx to be used next
        self.n_action = in_action #get number of actions as input parameter
        self.episode = 1.0 #counter used for decay, increased on every learning iteration 
        self.decay = i_decay
        self.b_Inverted = False
        self.b_NextIsRandom = False
    #Set to deterministic behavior. Can be used to manually turn off epsilon exploration
    def DoNotExplore(self):
        self.val = -1
        
    #Called before every action selection simulation
    #Epsilon greedy exploration
    def Explore(self):
        rand = np.random.uniform(0, 1) #get random number
        self.lastVal = copy.copy(self.nextVal) #save last used action index
        if (rand < self.epsilon or self.b_NextIsRandom == True): #random behavior if random number below current epsilon value
            self.nextVal = np.random.randint(0, self.n_action) #get a random adction idx within the range
            self.b_NextIsRandom = False
        else:
            self.nextVal = -1 #deterministic behavior, argmax will be used in net
        
#        if (self.lastVal != -1): #print indication of exploration to console 
        if (self.nextVal != -1): #print indication of exploration to console 
            print("Exploring action: " +str(self.nextVal) + " with current th_epsilon: " +str(self.epsilon))

    #Prepare value for action selection (next action needed)
    def SetActive(self):
        self.val = self.nextVal 
    #Prepare value for training (last action needed)
    def SetTraining(self):
        self.val = self.lastVal
    #Return constant value in every simulation step
    def __call__(self, t):
        if (abs(self.nextVal) >= self.n_action):
            print(self.nextVal)
        return self.nextVal #XXX was just vl
    #Called before every training, Update decaying epsilon parameter
    def OnTraining(self):
        self.episode += 1
        self.CalcEpsilon()
        
        if(self.nextVal >= 0): #TODO Document; rounding and negative explained
            self.nextVal = -self.nextVal - 0.15
            self.b_Inverted = True
        
    def AfterTraining(self):
        if(self.b_Inverted == True):
            self.b_Inverted = False
            self.nextVal = -self.nextVal + 0.15
    
    def SetUnsetDeterministic(self):
        temp = copy.deepcopy(self.epsilon_temp)
        self.epsilon_temp = copy.deepcopy(self.epsilon_init)
        self.epsilon_init = temp
        
    def ForceRandom(self):
        self.b_NextIsRandom = True

    def CalcEpsilon(self):
        self.epsilon = np.clip(self.epsilon_init/(float(self.episode)/(self.decay)), 0, self.epsilon_init) #/(150); epsilon init = 0.1

        
        

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
    
#A class that can be passed to a nengo node as input
#Used once training is completed in order to inhibit entire learning subnetwork
class NodeInhibitAlLTraining():
    def __init__ (self):
        self.b_DoInhibit = False #flag whether to inhibit not
    def __call__(self, t): #return inhibition value
        if (self.b_DoInhibit == True):
            return 1
        else:
            return 0
    def InhibitTrainingSubnetwork(self):
        self.b_DoInhibit = True
    def DoNotInhibitTrainingSubnetwork(self):
        self.b_DoInhibit = False
    def SwitchInhibit(self):
        self.b_DoInhibit = not self.b_DoInhibit
        
        
#A class that samples network values to print them to console
class NodeLogTraining():
    def __init__(self, sim_dt):
        self.QatStart = np.nan #q value at start of training
        self.QatEnd = np.nan #q value at end of simulation
        self.dt = 0.4 #
        self.sim_dt = sim_dt #simulation time step
        self.rewardProbed = np.nan #reward in network
        self.error = np.nan #error sample at start of training
        self.error_end = np.nan #error sample at end of trianing
    def __call__(self, t, x): #x is start time input 
        #sample at start of training (after dt) and limit to small window (uses sim_dt)
        if(t >= x[1]+self.dt and t < x[1]+self.dt+self.sim_dt):
            self.error = x[3] #error signal is [3] connected signal in network
            self.QatStart = x[0] #q utility is [0] conencted signal in network
        #sample on every call, last call will result in last value
        self.QatEnd = x[0] 
        self.rewardProbed = x[2]
        self.error_end =  x[3]
    #Print probed values to output
    def PrintTraining(self, reward, action):
        color1 = "\033[31m"
        color2 = "\033[32m"
        if (self.QatEnd < self.QatStart):
            color1, color2 = color2, color1
        print("Trained action \033[96m" + str(action) + "\033[0m with reward: \033[96m%.1f\033[0m/\033[96m%.1f\033[0m" % (reward, self.rewardProbed) + " (env./nengo)" +
                                    " and error %.2f / %.2f" % (self.error, self.error_end) + " | Result: Q-value changed from " + color1 + "%.3f" % self.QatStart + "\033[0m to " +
                                    color2 + "%.3f" % self.QatEnd + "\033[0m")

#A class to scale the error by different factors
#Factor 1: Implement time decaying learning rate by scaling error
#Factor 2: Implement inverse distance decaying (after some time, samples far away from vehicle will have been visited less frequently, therefore
#          their error should be valued higher as samples that are close to the start line)
class NodeErrorScaling():
    def __init__(self, error_decay):
        self.scale = 1.0 #value used for scaling
        self.nNumber = 0.0 #episode number
        self.f_maxDistance = 0.001 #value used to scale by previously known states
        self.error_decay = error_decay #time decay rate
        self.distance_array = [self.f_maxDistance] #last x values of best distance, used for moving average
        self.moving_size = 5 #size of moving average
    def __call__(self, t):
        return self.scale
    
    def SetScale(self, f_distance):
        self.nNumber += 1 #Update episode number on call
        f_DistanceScale = f_distance/self.f_maxDistance #Calculate new distance scale
        if(f_DistanceScale < 0.75): #if to close to starting line, scale error down
            self.scale = np.clip(f_DistanceScale / (self.nNumber/self.error_decay), 0, 1) #Implement time decay as well
        else:
            self.scale = np.clip(f_DistanceScale, 0, 1.1) #ignore time decay on unknown states
        print("The error scaling for this sample is: %.2f" % self.scale + " with current max distances: " + str(self.distance_array))
        
    #Add new max distance whenever race is restarted
    def OnRestart(self, f_newMax):
        if(f_newMax > 1): #sometimes the distance will falsely be close to 0
            if(len(self.distance_array) == self.moving_size): #see if moving average array is full
                self.distance_array = self.distance_array[1:] #if yes, erase first value in array
            self.distance_array.append(f_newMax)#add new value to array
        self.f_maxDistance = sum(self.distance_array)/len(self.distance_array) #Calculate new max distance

