#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 26 11:02:28 2018

@author: bzorn
"""

import nengo
import numpy as np
import matplotlib.pyplot as plt


import nengo_dl


def qnet_associative(b_Direct, signal, i_reward, i_time, i_output, n_action, label=''):
    param_neuron = nengo.neurons.LIF()
    if b_Direct == True:
        param_neuron = nengo.neurons.Direct()
        
    n_dim = len(signal)
#    n_action = 21 #currently hardcoded; should be parametrizable
    epsilon = 0
    
    
#    tau_short = 0.005
    tau_mid = 0.005
    tau_long = 0.01
    
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
        
        def func_afterT(t, x):
            #x[-1] is start time
            if (t > x[-1]+0.4 and t < x[-1]+0.5): #limits learning to window after 
                return x[:-1]
            else:
                return (len(x)-1)*[0]

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
        net.LearningConnections = [nengo.Connection(net.QEnsemble_In, net.QEnsembleArray_Out.input[n], function=lambda x: np.random.uniform(0, 0.1)
        , label='Learning Connection Action' + str(n), learning_rule_type=nengo.PES(learning_rate=0.001), synapse=tau_mid) for n in range(n_action)]
    

        [nengo.Connection(net.QEnsembleArray_Out.output[n], net.ActionSelection[n], label='Q utility to action'+str(n),
                          synapse=tau_long) for n in range(n_action)]
        nengo.Connection(net.eGreedyIn, net.ActionSelection[-1])
        
        net.Output = nengo.Node(output=i_output, size_in=n_action+1)#, size_out=None)
        nengo.Connection(net.ActionSelection, net.Output)
        
        
        net.Reward = nengo.Node(output=i_reward, size_in=None, size_out=n_action+1)
        net.Error = nengo.networks.EnsembleArray(n_neurons = 50, n_ensembles= n_action, ens_dimensions=1, label='Error calculation',
                                                 neuron_type=param_neuron)
        
        
        [nengo.Connection(net.Reward[-1], net.Error.input[n], transform=-1) for n in range(n_action)] #connect reward to each error ensemble
        [nengo.Connection(net.QEnsembleArray_Out.output[n], net.Error.input[n], transform=1) for n in range(n_action)]
        
        net.Delay = nengo.networks.EnsembleArray(n_neurons = 50, n_ensembles=n_action, ens_dimensions=1, label='Delayed and inhibited error')
        
        nengo.Connection(net.Error.output, net.Delay.input, synapse = tau_long)
        
        [nengo.Connection(net.Reward[n], net.Delay.ensembles[n].neurons, 
                          transform=-5*np.ones((net.Delay.ensembles[n].n_neurons, 1))) for n in range(n_action)]
    
        net.tStart = nengo.Node(output=i_time, size_in = None, size_out=1)
        net.LearnAfterT = nengo.Node(output=func_afterT, size_in = n_action+1, size_out = n_action)
        
#        [nengo.Connection(net.Delay.output[n], net.LearningConnections[n].learning_rule) for n in range(n_action)]
        [nengo.Connection(net.Delay.output[n], net.LearnAfterT[n]) for n in range(n_action)]
        nengo.Connection(net.tStart, net.LearnAfterT[-1])
        [nengo.Connection(net.LearnAfterT[n], net.LearningConnections[n].learning_rule) for n in range(n_action)]

        

#        net.Hard = nengo.Node(output=1)
#        nengo.Connection(net.Hard, net.ActionSelection[2], transform=-1) #add value to number 1 to increase 
#        nengo.Connection(net.Hard, net.Error.input[n], transform=-1) #add value to number 1 to increase 

        return net 
    
    
def input_function(t):
    return [0, 0.2, 0.1, 0.3, 0.7, 0, 0.1]

def reward_function(t):
    r = 22 * [1] #inverse one hot encoding
    r[-1] = 0.4
    if t > 1:
        r[10] = 0
#    r[10] = 0

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
    
    SNN_Q_ass = qnet_associative(False, signal, reward_function, 1.5, output.ProbeFunc, 21)
    nengo.rc.set('progress', 'progress_bar', 'nengo.utils.progress.TerminalProgressBar') #Terminal progress bar for inline
    
    with SNN_Q_ass:
        pr_epsiolon = nengo.Probe(SNN_Q_ass.ActionSelection)
        
        pr_selection = nengo.Probe(SNN_Q_ass.ActionSelection)
        pr_val_reps  = nengo.Probe(SNN_Q_ass.QEnsemble_In, synapse=0.1)
        
        pr_afterTrans = nengo.Probe(SNN_Q_ass.Error.output, synapse = 0.1)
        pr_afterDelay = nengo.Probe(SNN_Q_ass.Delay.output, synapse = 0.01)
       
        #    pr_Qvals = []
        #    [pr_Qvals.append(nengo.Probe(SNN_Q_ass.QEnsemble.output[n])) for n in range(n_dim)]
        pr_Qvals  = nengo.Probe(SNN_Q_ass.QEnsembleArray_Out.output, synapse=0.1)
                
#        sim = nengo.Simulator(SNN_Q_ass, progress_bar=True)
        sim = nengo_dl.Simulator(SNN_Q_ass, progress_bar=True)
        sim.run(2.5) 
        
        [plt.plot(sim.trange(), sim.data[pr_val_reps][:,n], label='input '+str(n)) for n in range(n_dim)]
        plt.legend(loc='best')
        plt.show()
        
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

        

    
    