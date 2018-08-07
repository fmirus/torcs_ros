#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Aug  7 09:34:02 2018

@author: bzorn
"""

import nengo
import numpy as np

#A gated memory for two time steps
def DoubleMemory(i_timeStart, i_signal, i_radius):
    #returns gate signals for both memory networks after dt and 2dt respectively
    def func_gate_double(t, x):
        dt = 0.2 #size of gated input
        if(t-x[0] > dt and t-x[0] < 2*dt):
            return [1, 0] #1 i
        elif(t-x[0] > dt and t-x[0] < 3*dt):
            return [0, 1]
        else:
            return [1, 1]
        
    
    def debug_time(t):

        if(t >= 4):
            return 4
        if(t >= 3):
            return 3
        if(t >= 2):
            return 2
        if(t >= 1):
            return 1
        else:
            return 0
    
    with nengo.Network() as double_mem_net:
        double_mem_net.GateSignal = nengo.Node(output = func_gate_double, size_in=1, size_out=2) #input t_start
        double_mem_net.Mem1 = nengo.networks.InputGatedMemory(n_neurons = 200, dimensions = len(i_signal), recurrent_synapse=0.01) #either state or q-value save
        for ens in double_mem_net.Mem1.all_ensembles: #set radius for memory 1 to input radius
            ens.radius = i_radius
        double_mem_net.Mem2 = nengo.networks.InputGatedMemory(n_neurons = 200, dimensions = len(i_signal), recurrent_synapse=0.01)  #set radius for memory 2 to input radius
        for ens in double_mem_net.Mem2.all_ensembles:
            ens.radius = i_radius
        nengo.Connection(double_mem_net.GateSignal[0], double_mem_net.Mem1.gate) #connect first gate signal (= second firing) to memory gate 1
        nengo.Connection(double_mem_net.GateSignal[1], double_mem_net.Mem2.gate) #connect second gate signal (= first firing) to memory gate 2
        double_mem_net.Signal = nengo.Node(output = i_signal) #create input node
        nengo.Connection(double_mem_net.Signal, double_mem_net.Mem1.input) #connect to memory 1 input
        nengo.Connection(double_mem_net.Mem1.output, double_mem_net.Mem2.input)#interconnect memories
        double_mem_net.output = nengo.Node(size_in=len(i_signal), size_out=len(i_signal)) #create a passthrough node for output 
        nengo.Connection(double_mem_net.Mem2.output, double_mem_net.output) #connect second memory to output
        
        #debug nodes to input start time and simulate several ros timesteps
        double_mem_net.DebugTime = nengo.Node(output = debug_time, size_in = 0, size_out=1)
        nengo.Connection(double_mem_net.DebugTime, double_mem_net.GateSignal, synapse=None) #synapse None or very low needed for ideal gating behavior (at least for debug)
        
        return double_mem_net
        
#A gated memory for one time step
def SingleMemory(i_timeStart, i_signal, i_radius):
    #return gate signal for memory network after dt with width dt
    def func_gate(t, x):
        dt = 0.2 #size of gated input
        if(t-x[0] > dt and t-x[0] < 2*dt):
            return 0 #1 i
        else:
            return 1
        
    def debug_time(t):

        if(t >= 4):
            return 4
        if(t >= 3):
            return 3
        if(t >= 2):
            return 2
        if(t >= 1):
            return 1
        else:
            return 0
        
    with nengo.Network() as mem_net:
        mem_net.GateSignal  = nengo.Node(output = func_gate, size_in=1, size_out=1) #input t_start
        mem_net.Mem = nengo.networks.InputGatedMemory(n_neurons = 200, dimensions = len(i_signal), recurrent_synapse=0.01) #either state or q-value save
        for ens in mem_net.Mem.all_ensembles: #set radius for memory to input radius
            ens.radius = i_radius
        nengo.Connection(mem_net.GateSignal, mem_net.Mem.gate) #connect gate signal to gate
        mem_net.Signal = nengo.Node(output = i_signal) #create input node
        nengo.Connection(mem_net.Signal, mem_net.Mem.input) #connect to memory
        mem_net.output = nengo.Node(size_in=len(i_signal), size_out=len(i_signal)) #create a passthrough node for output
        nengo.Connection(mem_net.Mem.output, mem_net.output) #connect to memory
        
        #debug nodes for input start time and simulate several ros timesteps
        mem_net.DebugTime = nengo.Node(output = debug_time, size_in = 0, size_out=1)
        nengo.Connection(mem_net.DebugTime, mem_net.GateSignal, synapse=None) #synapse None or very low needed for ideal gating behavior (at least for debug)
        
        return mem_net