#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Aug  6 10:55:31 2018

@author: bzorn
"""

import nengo
import numpy as np
import matplotlib.pyplot as plt




    
    
def create_action_selection_net(b_Direct, signal, i_reward, i_time, i_epsilon, i_inhibit, i_output, n_action, i_radius, tau_mid, tau_long, f_learningRate=0.001 ,label=''):
    param_neuron = nengo.neurons.LIF()
    if b_Direct == True:
        param_neuron = nengo.neurons.Direct()
        
    n_dim = len(signal)


    with nengo.Network(label=label) as net: 
        #action selection with decaying epsilon exploration
        def func_epsilonMax(t, x):
#            if x[-1] >= epsilon/(t/1000): 
            if x[-1] < 0: #mark for 
                idx = np.argmax(x[:-1])
            else: #explore
                idx = int(round(x[-1]))
            retVec = np.zeros(n_action)
            retVec[idx] = 1 
            retVec = retVec.tolist()
            retVec.append(x[idx]) #returns Q-value associated to chosen action
            return retVec
        
        def func_afterT(t, x):
            #x[-1] is start time
            if (t > x[-1]+0.4 and t < x[-1]+0.5): #limits learning to window after 
                return x[:-1]
            else:
                return (len(x)-1)*[0]
        if (signal != None):
            net.input = nengo.Node(output=signal, size_in=None, size_out=n_dim)
        else:
            net.input = nengo.Node(size_in=n_dim, size_out=n_dim)
        net.eGreedyIn = nengo.Node(output=i_epsilon, label='epsilon greedy input')
        
        #Create an ensemble, each dimension encoding one rangefinder sensor
        net.QEnsemble_In = nengo.Ensemble(n_neurons = 100, dimensions = n_dim, neuron_type=param_neuron, label='input encoding', radius=1) 
        nengo.Connection(net.input, net.QEnsemble_In) #connect input to representation

        #Create an array of ensembles, each representing one action
        net.QEnsembleArray_Out = nengo.networks.EnsembleArray(n_neurons = 100, n_ensembles=n_action, ens_dimensions=1, 
                                                                    label='Q Values',neuron_type=param_neuron, radius=i_radius) 
       
        #Connect encoding to Q-values with a learnable connection per action
        #Initalize Q population with a low random reward
        net.LearningConnections = [nengo.Connection(net.QEnsemble_In, net.QEnsembleArray_Out.input[n], function=lambda x: 1,#np.random.uniform(0, 0.1), 
                                                    label='Learning Connection Action' + str(n), learning_rule_type=nengo.PES(learning_rate=f_learningRate,
                                                                                            pre_synapse=nengo.synapses.Lowpass(tau=0.05)), #default is tau=0.005
                                                    synapse=tau_mid) for n in range(n_action)]
    
        
        #Create a node that implements an epsilon-exploration argmax function 
        #Last value is outputted Q-value, every other output is one-hot encoded selected action with action idx
        net.ActionSelection = nengo.Node(output=func_epsilonMax, size_in=n_action+1, size_out=n_action+1, label='Action selection and exploration') 
        #Connect Q values and epsilon choice to action selection
        [nengo.Connection(net.QEnsembleArray_Out.output[n], net.ActionSelection[n], label='Q utility to action'+str(n),
                          synapse=tau_long) for n in range(n_action)]
        nengo.Connection(net.eGreedyIn, net.ActionSelection[-1])
        
        net.Output = nengo.Node(output=i_output, size_in=n_action+1, size_out=None, label='Output prober')
        nengo.Connection(net.ActionSelection, net.Output)
        
        return net
    
def create_error_net_associative(net, i_reward, n_action,param_neuron, tau_mid, tau_long):
    with net:
        with nengo.Network(label='Error network') as net.errorA_net:
            net.errorA_net.Reward = nengo.Node(output=i_reward, size_in=None, size_out=n_action+1)
            net.errorA_net.Error = nengo.networks.EnsembleArray(n_neurons = 50, n_ensembles= n_action, ens_dimensions=1, label='Error calculation',
                                                         neuron_type=param_neuron)
            [nengo.Connection(net.errorA_net.Reward[-1], net.errorA_net.Error.input[n], transform=-1) for n in range(n_action)] #connect reward to each error ensemble
            net = connect_to_error_net_associative(net, n_action, tau_mid, tau_long)
            return net
            
def connect_to_error_net_associative(net, n_action, tau_mid, tau_long):
    with net:
        [nengo.Connection(net.QEnsembleArray_Out.output[n], net.errorA_net.Error.input[n], transform=1) for n in range(n_action)]
        return net
    
def create_learning_net(net, i_time, i_inhibit, n_action, tau_mid, tau_long):
    with net:
        with nengo.Network(label='learning network') as net.learning_net: 
            def func_afterT(t, x):
                #x[-1] is start time
                if (t > x[-1]+0.4 and t < x[-1]+0.5): #limits learning to window after 
                    return x[:-1]
                else:
                    return (len(x)-1)*[0]
            net.learning_net.Delay = nengo.networks.EnsembleArray(n_neurons = 50, n_ensembles=n_action, ens_dimensions=1, label='Delayed and inhibited error')
            net.learning_net.tStart = nengo.Node(output=i_time, size_in = None, size_out=1)
            net.learning_net.LearnAfterT = nengo.Node(output=func_afterT, size_in = n_action+1, size_out = n_action)
    
            [nengo.Connection(net.learning_net.Delay.output[n], net.learning_net.LearnAfterT[n]) for n in range(n_action)]
            nengo.Connection(net.learning_net.tStart, net.learning_net.LearnAfterT[-1], synapse=None) #achieve ideal gating for learning
    
            net.learning_net.InhibitAllTraining = nengo.Node(output=i_inhibit, size_in=None, size_out=1)
            [nengo.Connection(net.learning_net.InhibitAllTraining, net.learning_net.Delay.ensembles[n].neurons, 
                              transform=-10*np.ones((net.learning_net.Delay.ensembles[n].n_neurons, 1))) for n in range(len(net.learning_net.Delay.ensembles))]
            net = connect_to_learning_net(net, n_action, tau_mid, tau_long)
            return net
            
        
def connect_to_learning_net(net, n_action, tau_mid, tau_long):
    with net:
        nengo.Connection(net.errorA_net.Error.output, net.learning_net.Delay.input, synapse = tau_long)
        [nengo.Connection(net.errorA_net.Reward[n], net.learning_net.Delay.ensembles[n].neurons, 
                          transform=-5*np.ones((net.learning_net.Delay.ensembles[n].n_neurons, 1))) for n in range(n_action)]
    
        [nengo.Connection(net.learning_net.LearnAfterT[n], net.LearningConnections[n].learning_rule) for n in range(n_action)]
        [nengo.Connection(net.learning_net.InhibitAllTraining, net.errorA_net.Error.ensembles[n].neurons, 
                      transform=-10*np.ones((net.errorA_net.Error.ensembles[n].n_neurons, 1))) for n in range(len(net.errorA_net.Error.ensembles))]
        return net