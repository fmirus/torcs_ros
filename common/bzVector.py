#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 23 15:51:19 2018

@author: bzorn
"""

#Vector class to manually store translations
class vec3():
    def __init__(self, i_x=0, i_y=0, i_z=0):
        self.x,self.y,self.z = i_x,i_y,i_z
    def Set(self, msg_frame):
        self.x = msg_frame.transform.translation.x  
        self.y = msg_frame.transform.translation.y
        self.z = msg_frame.transform.translation.z
    def __getitem__(self, i):
        vec = [self.x, self.y, self.z]
        return vec[i]
#Vector class to manually store quaternion orientations
class vec4():
    def __init__(self, i_x=0, i_y=0, i_z=0, i_w=1):
        self.x,self.y,self.z,self.w = i_x,i_y,i_z,i_w
    def Set(self, msg_frame):
        try:
            self.x = msg_frame.transform.rotation.x  
            self.y = msg_frame.transform.rotation.y
            self.z = msg_frame.transform.rotation.z
            self.w = msg_frame.transform.rotation.w
        except:
            self.x = msg_frame.pose.orientation.x  
            self.y = msg_frame.pose.orientation.y
            self.z = msg_frame.pose.orientation.z
            self.w = msg_frame.pose.orientation.w
    def __getitem__(self, i):
        vec = [self.x, self.y, self.z, self.w]
        return vec[i]