#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 26 11:02:28 2018

@author: bzorn
"""

import nengo
import numpy as np
import matplotlib.pyplot as plt

def qnet_associative(b_Direct, signal, i_output, label=''):
    param_neuron = nengo.neurons.LIF()
    if b_Direct == True:
        param_neuron = nengo.neurons.Direct()
        
    n_dim = len(signal)
    n_action = 21 #currently hardcoded; should be parametrizable
    epsilon = 0
    
    
    with nengo.Network(label=label) as net: 
        #action selection with decaying epsilon exploration
        def func_epsilonMax(t, x):
            if x[-1] >= epsilon/(t/1000): 
                idx = np.argmax(x[:-1])
            else: #explore
                idx = np.random.randint(0, n_action)
            retVec = np.zeros(n_action)
            retVec[idx] = 1
            retVec = retVec.tolist()
            retVec.append(x[idx])
            return retVec

        net.stateIn = nengo.Node(output=signal, size_in=None, size_out=n_dim)
        net.eGreedyIn = nengo.Node(output=lambda t: np.random.uniform(0,1), label='epsilon greedy input')
        
        #intermediate ensemble needed as decoders of this are what are learned
        #can be ensemble array or ensemble dependent on whether you want to imply the correlation between the signals
        #which should be given for the rangefinders
        net.QEnsemble_In = nengo.Ensemble(n_neurons = 100, dimensions = n_dim, neuron_type=param_neuron, label='input encoding') 
        
        net.QEnsembleArray_Out = nengo.networks.EnsembleArray(n_neurons = 100, n_ensembles=n_action, ens_dimensions=1, 
                                                                    label='Q Values',neuron_type=param_neuron) #one ensemble for each action
        #last value is outputted Q-value, every other output is one-hot encoded selected action with action idx
        net.ActionSelection = nengo.Node(output=func_epsilonMax, size_in=n_action+1, size_out=n_action+1, label='Action selection and exploration') 
        nengo.Connection(net.stateIn, net.QEnsemble_In)
        #initalize Q population with fixed low reward for all (alternative: low but random )
        net.LearningConnections = [nengo.Connection(net.QEnsemble_In, net.QEnsembleArray_Out.input[n], function=lambda x: np.random.uniform(0, 0.01), label='Learning Connection Action' + str(n)) for n in range(n_action)]

        [nengo.Connection(net.QEnsembleArray_Out.output[n], net.ActionSelection[n], label='Q utility to action'+str(n)) for n in range(n_action)]
        nengo.Connection(net.eGreedyIn, net.ActionSelection[-1])
        
        net.Output = nengo.Node(output=i_output, size_in=n_action+1)#, size_out=None)
        nengo.Connection(net.ActionSelection, net.Output)
        
        #fix that one action is always selected
        net.Hard = nengo.Node(output=1)
        nengo.Connection(net.Hard, net.ActionSelection[2]) #add value to number 1 to increase 
        return net 
    
    
def input_function(t):
    return [0, 0.2, 0.1, 0.3, 0.7, 0, 0.1]



if __name__ == "__main__":
    signal = input_function(0)
    
    SNN_Q_ass = qnet_associative(True, signal)
    nengo.rc.set('progress', 'progress_bar', 'nengo.utils.progress.TerminalProgressBar') #Terminal progress bar for inline
    
    with SNN_Q_ass:
        pr_epsiolon = nengo.Probe(SNN_Q_ass.ActionSelection)
        
        pr_selection = nengo.Probe(SNN_Q_ass.ActionSelection)
        pr_val_reps  = nengo.Probe(SNN_Q_ass.QEnsemble_In, synapse=0.01)
        
        #    pr_Qvals = []
        #    [pr_Qvals.append(nengo.Probe(SNN_Q_ass.QEnsemble.output[n])) for n in range(n_dim)]
        pr_Qvals  = nengo.Probe(SNN_Q_ass.QEnsembleArray_Out.output, synapse=0.01)
        
        sim = nengo.Simulator(SNN_Q_ass, progress_bar=True)
        sim.run(1) 
        
        [plt.plot(sim.trange(), sim.data[pr_val_reps][:,n], label='input '+str(n)) for n in range(n_dim)]
        plt.legend(loc='best')
        plt.show()
        
        [plt.plot(sim.trange(), sim.data[pr_Qvals][:,n], label='action '+str(n)) for n in range(n_action)]
        plt.legend(loc='best', ncol=4)
        plt.show()
    
    
        plt.plot(sim.trange(), sim.data[pr_selection])
        plt.legend(loc='best')




    
    