#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 27 12:57:53 2018

@author: bzorn
"""

import yaml


def readTrajectoryParams(src_path):
    src_path += "/torcs_ros_trajectory_gen/include/torcs_ros_trajectory_gen/trajectory_params.yaml"
    
    with open(src_path, 'r') as ymlfile:
        params = yaml.load(ymlfile)
    	return [params['f_longitudinalDist'], params['f_lateralDist'], params['n_amount']]

def readSavedTrajectoryParams(path):
    with open(path, 'r') as ymlfile:
        params = yaml.load(ymlfile)
    	return [params['TRAJECTORY_PARAMETERS']['longitudinal_distance'], params['TRAJECTORY_PARAMETERS']['lateral_distance'], 
             params['TRAJECTORY_PARAMETERS']['total_number_actions']]
    
        
    
#definition here so that it has to only be changed once across files
def calcTrajectoryAmount(n_amount):
    return 2*n_amount+1

def readVisualize(src_path):
    src_path += "/torcs_ros_trajectory_gen/include/torcs_ros_trajectory_gen/trajectory_params.yaml"
    with open(src_path, 'r') as ymlfile:
        params = yaml.load(ymlfile)
    	return params['b_visualizeAll']
    

def readNengoHyperparams(src_path):
    src_path += "/torcs_ros_trajectory_selection/include/torcs_ros_trajectory_selection/simulation_params.yaml"
    with open(src_path, 'r') as ymlfile:
        params = yaml.load(ymlfile)
    return [params['f_epsilon_init'], params['f_decay_factor'], params['f_learning_rate'], params['a_scanTrack']]

def readsavedNengoHyperparams(path):
    with open(path, 'r') as ymlfile:
        params = yaml.load(ymlfile)
    return [params['LEARNING_PARAMETERS']['epsilon_init'], params['LEARNING_PARAMETERS']['decay'], 
            params['LEARNING_PARAMETERS']['learning_rate'], params['scan_sensors']]

def saveYAML(dictionary, path):
        #save as yaml file
        with open(path, 'w') as yamlfile:
            yaml.dump(dictionary, yamlfile, default_flow_style=False)
            
def readConfig(path):
    with open(path, 'r') as ymlfile:
        params = yaml.load(ymlfile)
    	return [params['data_log'], params['nengo_save'], params['nengo_load'], params['nengo_weigths_path'], params['directory']]
    
def readConfigSrc(src_path):
    src_path = src_path[:-14] + "nengo_parameters/config.yaml"
    with open(src_path, 'r') as ymlfile:
        params = yaml.load(ymlfile)
    	return [params['data_log'], params['nengo_save'], params['nengo_load'], params['nengo_weigths_path'], params['directory']]