#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 23 15:34:21 2018

@author: bzorn
"""

import numpy as np
import rospy

from geometry_msgs.msg import TwistStamped, Vector3Stamped, PoseStamped, Point, PointStamped, TransformStamped, Quaternion
from visualization_msgs.msg import Marker
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path
from torcs_msgs.msg import TORCSCtrl, TORCSSensors

#add common folder to path; have to replace this later with rospkg when launched with roslaunch 
import os
import sys
cwd = os.getcwd()
sys.path.append(cwd[:-29] + "common")

from bzVector import vec3, vec4#2d and 3d vector definitions
from bzGeometricFuncs import BaseLinkToTrajectory
from bzGearLUT import SetGearFromLUT


class FollowTrajectory():
    def __init__(self, trajectory_topic="/torcs_ros/trajectorySelected", ctrl_topic = "/torcs_ros/ctrl_cmd", 
                 frame_topic="/tf", sensors_topic="/torcs_ros/sensors_state"):
        
        self.ros_trans = vec3()
        self.ros_rot = vec4()
        self.time = rospy.Time.now()
        
        self.trajectory = Path()
        self.sen_angle = 0 #delta angle to mid line
        self.sen_trackpos = 0 #delta x to mid line normalized
        self.sen_rpm = 0
        self.sen_gear = 0
        
        self.ctrl_accel = 0
        self.ctrl_brake = 0
        self.ctrl_steering = 0
        self.ctrl_gear = 0
        
        self.param_kappa = 0.75 #value used to adjust desired heading to 
        self.param_steerLock = np.deg2rad(21) #max. steerLock in torcs/data/cars/tbt1 is 21 deg
        
        
        rospy.wait_for_message(frame_topic, TFMessage) #wait until frame is published
        rospy.wait_for_message(sensors_topic, TORCSSensors) #also wait until sensors are published 
        
 
        self.sub_trajectory = rospy.Subscriber(trajectory_topic, Path, self.trajectory_callback)
        self.sub_sensors = rospy.Subscriber(sensors_topic, TORCSSensors, self.sensors_callback)
        self.sub_frame = rospy.Subscriber(frame_topic, TFMessage, self.frame_callback, queue_size=1)
        
        self.pub_ctrl = rospy.Publisher(ctrl_topic, TORCSCtrl, queue_size=10)
        

        
    def trajectory_callback(self, msg_trajectory):
        self.trajectory = msg_trajectory
        #path with poses here
        
    def sensors_callback(self, msg_sensors):
        self.sen_angle = msg_sensors.angle #angle in rad
        self.sen_trackpos = msg_sensors.trackPos #normalized trackpos
        self.sen_rpm = msg_sensors.rpm
        self.sen_gear = msg_sensors.gear
    def frame_callback(self, msg_frame):
        frame_idx = 0 #index to identify the proper transformation if there is more than the world and base_link frames avalaible
        b_frameHasBeenFound = False #flag to show if transformation has been found
        #identify transformation
        for frame in msg_frame.transforms:
            if frame.header.frame_id == "world" and frame.child_frame_id == "base_link":
                b_frameHasBeenFound = True #set flag that a valid transformation has been published
                break;
            ++frame_idx    
        if b_frameHasBeenFound == True: #ensure there is data to use
            self.newest_ros_transform = msg_frame.transforms[frame_idx] #get transform from message
            self.time = msg_frame.transforms[frame_idx].header.stamp #get time from message
            self.ros_trans.Set(msg_frame.transforms[frame_idx]) #Get translation from published ros message
            self.ros_rot.Set(msg_frame.transforms[frame_idx]) #Get rotation from published ros message
            self.calculate_ctrl_commands()
            
            
    def calculate_ctrl_commands(self):
        msg_ctrl = TORCSCtrl()
        
#        f_distToBaselink = BaseLinkToTrajectory(self.ros_trans.x, self.ros_trans.y, self.trajectory.poses) #needed for actual control
        
        
        self.ctrl_steering = (self.sen_angle - self.sen_trackpos*self.param_kappa)/self.param_steerLock
        
        if (self.sen_rpm < 4000 or self.sen_gear == 0):
            self.ctrl_accel = 0.2
        else:
            self.ctrl_accel = 0
        self.ctrl_gear = SetGearFromLUT(self.sen_gear, self.sen_rpm)
#        print(self.gear)
        msg_ctrl.accel = self.ctrl_accel
        msg_ctrl.steering = self.ctrl_steering
        msg_ctrl.brake = self.ctrl_brake #set to 0
        msg_ctrl.gear = self.ctrl_gear
        
        self.pub_ctrl.publish(msg_ctrl)
         
    def pathTo2DPosition(self, path):
        positions2D = []
        for pose in path.poses:
            positions2D.append([pose.position.x, pose.position.y])
        positions2D = np.asarray(positions2D)
            
if __name__ == "__main__":
    rospy.init_node("Trajectory_Follow_Control")
    Controller = FollowTrajectory()
    rospy.spin()