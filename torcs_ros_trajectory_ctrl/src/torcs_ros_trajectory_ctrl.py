#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 23 15:34:21 2018

@author: bzorn
"""

import numpy as np
import rospy
import tf


from geometry_msgs.msg import TwistStamped, Vector3Stamped, PoseStamped, Point, PointStamped, TransformStamped, Quaternion
from visualization_msgs.msg import Marker
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path
from torcs_msgs.msg import TORCSCtrl, TORCSSensors
from std_msgs.msg import Bool

#add common folder to path; have to replace this later with rospkg when launched with roslaunch 
import os
import sys
cwd = os.getcwd()
sys.path.append(cwd[:-29] + "common")

from bzVector import vec3, vec4#2d and 3d vector definitions
from bzGeometricFuncs import BaseLinkToTrajectory
from bzGearLUT import SetGearFromLUT
import bzConsoleIndicators

class FollowTrajectory():
    def __init__(self, trajectory_topic="/torcs_ros/trajectorySelected", ctrl_topic = "/torcs_ros/ctrl_cmd", 
                 frame_topic="/tf", sensors_topic="/torcs_ros/sensors_state"):
        

        #### subscription parameters ####
        self.ros_trans = vec3() #global position of baselink frame
        self.ros_rot = vec4() #global orientation of baselink frame
        
        self.trajectory = Path() #selected trajectory by q-system
        self.sen_angle = 0 #delta angle to mid line
        self.sen_trackpos = 0 #delta x to mid line normalized
        self.sen_rpm = 0 #engine rmp
        self.sen_gear = 0 #current gear
        
        #### publication parameters ####
        self.ctrl_accel = 0 #desired accel signal [calculated for low constant speed]
        self.ctrl_brake = 0 #desired brake signal [fixed at 0]
        self.ctrl_steering = 0 #desired steering [calculated from trajectory]
        self.ctrl_gear = 0 #desired gear [calculated from look up table]
        self.needForAction_msg = Bool()
        
        #### calculation parameters and variables #### 
        self.param_kappa = 0.75 #value used to adjust desired heading to 
        self.param_steerLock = np.deg2rad(21) #max. steerLock in torcs/data/cars/tbt1 is 21 deg
        self.trajectory_rot = vec4() #desired orientation from trajectory 
       
        #### ensure all dependent nodes are up and running ####
        rospy.wait_for_message(frame_topic, TFMessage) #wait until frame is published
        rospy.wait_for_message(sensors_topic, TORCSSensors) #also wait until sensors are published 
#        rospy.wait_for_message(trajectory_topic, Path) #and wait for trajectory ()
        
        #### subscriptions ####
        self.sub_trajectory = rospy.Subscriber(trajectory_topic, Path, self.trajectory_callback)
        self.sub_sensors = rospy.Subscriber(sensors_topic, TORCSSensors, self.sensors_callback)
        self.sub_frame = rospy.Subscriber(frame_topic, TFMessage, self.frame_callback, queue_size=1)
#        self.sub_tf_listener = tf.TransformListener()
        
        
        #### publications ####
        self.pub_ctrl = rospy.Publisher(ctrl_topic, TORCSCtrl, queue_size=10)
        self.pub_needTrajectory = rospy.Publisher("/torcs_ros/ctrl_signal_action", Bool, queue_size=1)

  
                


    #get trajectory
    def trajectory_callback(self, msg_trajectory):
        self.trajectory = msg_trajectory
        self.needForAction_msg.data = False
        
    #get sensor values
    def sensors_callback(self, msg_sensors):
        self.sen_angle = msg_sensors.angle #angle in rad
        self.sen_trackpos = msg_sensors.trackPos #normalized trackpos
        self.sen_rpm = msg_sensors.rpm #rpm [no unit]
        self.sen_gear = msg_sensors.gear #current gear
        
    #get baselink frame position and orientation 
    def frame_callback(self, msg_frame):
        frame_idx = 0 #index to identify the proper transformation if there is more than the world and base_link frames avalaible
        b_frameHasBeenFound = False #flag to show if transformation has been found
        
#        [tf_trans, tf_rot] = self.sub_tf_listener.lookupTransform('world', 'base_link', rospy.Time(0)) #are identical
#        print(self.ros_trans.x, tf_trans[0])
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
            # If new frame information has been published, new commands have to be calculated
            self.calculate_ctrl_commands() 
            
    #calculate and publish control commands
    def calculate_ctrl_commands(self):
        if (len(self.trajectory.poses) > 0): #ensure this callback does not happen before the first trajectory has been published
            msg_ctrl = TORCSCtrl() #message container
            
            [f_distToTraj, idx_poseHeading, f_distToEnd] = BaseLinkToTrajectory(
                    self.ros_trans.x, self.ros_trans.y, self.trajectory.poses) #Calculate baselink to trajectory information
            self.trajectory_rot.Set(self.trajectory.poses[idx_poseHeading]) #Convert msg type to vec4() quatenrion
            f_deltaHeading = self.ComputeAngleDifference(self.ros_rot, self.trajectory_rot) #difference in desired orientation to current baselink orientation
            
    #        print(f_distToEnd)
            print(f_deltaHeading, f_distToTraj, f_distToEnd)
    
            ## f_distToBaselink und f_deltaHeading instead of trackpos and angle 
    
#            self.ctrl_steering = (self.sen_angle - self.sen_trackpos*self.param_kappa)/self.param_steerLock #control towards midline
            
            #dist to trajectory has to have a sign
            
            self.ctrl_steering = (f_deltaHeading + f_distToTraj/20*self.param_kappa)/self.param_steerLock
            
            #limit acceleration signal to achieve low constant speeds
            if (self.sen_rpm < 4000 or self.sen_gear == 0): #allow high enough rpm if car is in neutral gear (start of race)
                self.ctrl_accel = 0.2
            else:
                self.ctrl_accel = 0
                
            self.ctrl_gear = SetGearFromLUT(self.sen_gear, self.sen_rpm) #gear lookup
#            if (abs(self.sen_trackpos) > 1):
#                msg_ctrl.meta = 1
#            else:
#                msg_ctrl.meta = 0
            #add control data to message container
            msg_ctrl.accel = self.ctrl_accel
            msg_ctrl.steering = self.ctrl_steering
            msg_ctrl.brake = self.ctrl_brake #set to 0
            msg_ctrl.gear = self.ctrl_gear
            
            self.pub_ctrl.publish(msg_ctrl) #publish
            
            if (f_distToEnd < 1):
                self.needForAction_msg.data = True
#            else:
#                self.needForAction_msg.data = False
            self.pub_needTrajectory.publish(self.needForAction_msg)
    #        bzConsoleIndicators.IamWorking() #indicate that node is running and messages are sent
        else:
            self.needForAction_msg.data = True
            self.pub_needTrajectory.publish(self.needForAction_msg)
            
            
    #Difference in angle [in rad] between vehicle orientation (baselink) to closest point on trajectory
    def ComputeAngleDifference(self, quad_baselink, quad_trajectory):
        [roll_bl, pitch_bl, yaw_bl] = tf.transformations.euler_from_quaternion(quad_baselink)
        [roll_tj, pitch_tj, yaw_tj] = tf.transformations.euler_from_quaternion(quad_trajectory)
        return yaw_tj-yaw_bl
        
            
if __name__ == "__main__":
    rospy.init_node("Trajectory_Follow_Control")
    Controller = FollowTrajectory()
    rospy.spin()