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
    
    
#definition here so that it has to only be changed once across files
def calcTrajectoryAmount(n_amount):
    return 2*n_amount+1

def readVisualize(src_path):
    src_path += "/torcs_ros_trajectory_gen/include/torcs_ros_trajectory_gen/trajectory_params.yaml"
    with open(src_path, 'r') as ymlfile:
        params = yaml.load(ymlfile)
    	return params['b_visualizeAll']