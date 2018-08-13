#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 26 11:02:28 2018

@author: bzorn
"""

import nengo
import numpy as np
import matplotlib.pyplot as plt
from nengo_nets_subnetworks import create_learning_net, connect_to_learning_net, create_action_selection_net, create_error_net_associative


import nengo_dl

def qnet_associative(b_Direct, signal, i_reward, i_time, i_epsilon, i_inhibit, i_output, i_trainingProbe, i_errorScale, n_action, f_learningRate=0.001 ,label=''):
        param_neuron = nengo.neurons.LIF() #default neuron is leaky integrate and fire
        if b_Direct == True: #set output to direct //depreceated since learning has been added
            param_neuron = nengo.neurons.Direct()
                    
        #time constant definitions
        tau_mid = 0.005 
        tau_long = 0.01
        q_radius = 2.25
        
        #create action selection net (state---->encoding---learnin_rule--->Q-values--->epsilon-greedy_argmax-->selected_action)
        net = create_action_selection_net(b_Direct, signal, i_reward, i_time, i_epsilon, i_inhibit, i_output, 
                                          n_action, q_radius, tau_mid, tau_long,
                                          f_learningRate,label='')
        with net: 
            
            #create error calculation net for associative learning (Q-values ----> Error calculation )
            #                                                       Reward   ------^
            net = create_error_net_associative(net, i_reward, n_action, param_neuron, q_radius, tau_mid, tau_long)
            #create learning net (for associative learning)                             
            #                             (Error ----> Delay ----> Only learn in Time Window ---x learning_rule)
            # One hot encoding ...inhibit...^                                               
            net = create_learning_net(net, i_time, i_inhibit, i_trainingProbe, i_errorScale, n_action, tau_mid, tau_long);

        #        net.Hard = nengo.Node(output=1)
#        nengo.Connection(net.Hard, net.ActionSelection[2], transform=-1) #add value to number 1 to increase 
#        nengo.Connection(net.Hard, net.Error.input[n], transform=-1) #add value to number 1 to increase 

        return net 
    
    
    
def input_function(t):
    return [0, 0.2, 0.1, 0.3, 0.7, 0, 0.1]



#This function can be used to validate an output
#Set reward r[-1] to a value (e.g. 0.4)
#Set all action inhibitions to 1
#Set one action inhibition to 0. It will start learning after t_start = 1.5 (can be changed in the qnet_associative call)
def reward_function(t):
    r = 22 * [1] #inverse one hot encoding
    r[-1] = 0.4
#    if t > 1:
#        r[10] = 1
    r[10] = 0
    return r


class RNodeOutputProber():
    def __init__ (self, n_action):
        self.probe_vals = [0 for n in range(n_action+1)] #init array, one entry each for one-hot encoding and one for the highest q-value
        self.time_val = 0 #can save current simulation time as well, not used currently
    def ProbeFunc(self, t, x):
        self.probe_vals = x #update members to nengo output
        self.time_val = t 
        return self.probe_vals

#function used for debugging and visualization purposes
if __name__ == "__main__":
    n_dim = 7
    n_action = 21
    signal = input_function(0)
    reward = reward_function(0)
    output = RNodeOutputProber(n_action)
    
    SNN_Q_ass = qnet_associative(False, signal, reward_function, 1.5, np.random.uniform(0, 1), 0, output.ProbeFunc, 21)
    nengo.rc.set('progress', 'progress_bar', 'nengo.utils.progress.TerminalProgressBar') #Terminal progress bar for inline
    
    with SNN_Q_ass:
        pr_epsiolon = nengo.Probe(SNN_Q_ass.ActionSelection)
        
        pr_selection = nengo.Probe(SNN_Q_ass.ActionSelection)
        pr_val_reps  = nengo.Probe(SNN_Q_ass.QEnsemble_In, synapse=0.1)
        
        pr_afterTrans = nengo.Probe(SNN_Q_ass.errorA_net.Error.output, synapse = 0.1)
        pr_afterDelay = nengo.Probe(SNN_Q_ass.learning_net.Delay.output, synapse = 0.01)
       
        #    pr_Qvals = []
        #    [pr_Qvals.append(nengo.Probe(SNN_Q_ass.QEnsemble.output[n])) for n in range(n_dim)]
        pr_Qvals  = nengo.Probe(SNN_Q_ass.QEnsembleArray_Out.output, synapse=0.1)
                
#        sim = nengo.Simulator(SNN_Q_ass, progress_bar=True)
        sim = nengo_dl.Simulator(SNN_Q_ass, progress_bar=True)
        sim.run(2.5) 
        
#        [plt.plot(sim.trange(), sim.data[pr_val_reps][:,n], label='input '+str(n)) for n in range(n_dim)]
#        plt.legend(loc='best')
#        plt.show()
        
        [plt.plot(sim.trange(), sim.data[pr_Qvals][:,n], label='action '+str(n)) for n in range(n_action)]
        plt.legend(loc='best', ncol=4)
        plt.show()
    
    
        plt.plot(sim.trange(), sim.data[pr_selection])
        plt.legend(loc='best')
        plt.show()
        
        plt.plot(sim.trange(), sim.data[pr_afterTrans][:,10], label='Error')
#        plt.plot(sim.trange(), sim.data[pr_Qvals][:,0])
        plt.plot(sim.trange(), sim.data[pr_afterDelay][:,10], label='Delayed/Inhibited Error')
        plt.plot(sim.trange(), sim.data[pr_Qvals][:,10], label = 'Calculated output')
        plt.legend(loc='best')
        

        

    
    