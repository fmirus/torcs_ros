#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 23 15:34:21 2018

@author: bzorn
"""

import numpy as np
import rospy
import subprocess
import tf


from geometry_msgs.msg import TwistStamped, Vector3Stamped, PoseStamped, Point, PointStamped, TransformStamped, Quaternion
from visualization_msgs.msg import Marker
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path
from torcs_msgs.msg import TORCSCtrl, TORCSSensors
from std_msgs.msg import Bool
from std_msgs_stamped.msg import BoolStamped
#add common folder to path; have to replace this later with rospkg when launched with roslaunch 
import os
import sys
#cwd = os.getcwd()
#sys.path.append(cwd[:-29] + "common")

import rospkg
cwd = rospkg.RosPack().get_path('torcs_ros_trajectory_ctrl')
sys.path.append(cwd[:-25] + "common")
print(cwd[:-25] + "common")

from bzVector import vec3, vec4#2d and 3d vector definitions
from bzGeometricFuncs import BaseLinkToTrajectory
from bzGearLUT import SetGearFromLUT
import bzConsoleIndicators

class FollowTrajectory():
    def __init__(self, trajectory_topic="/torcs_ros/trajectorySelected", ctrl_topic = "/torcs_ros/ctrl_cmd", 
                 frame_topic="/tf", sensors_topic="/torcs_ros/sensors_state", speed_topic = "/torcs_ros/speed"):
        

        #### subscription parameters ####
        self.ros_trans = vec3() #global position of baselink frame
        self.ros_rot = vec4() #global orientation of baselink frame
        
        self.trajectory = Path() #selected trajectory by q-system
        self.sen_angle = 0 #delta angle to mid line
        self.sen_trackpos = 0 #delta x to mid line normalized
        self.sen_rpm = 0 #engine rmp
        self.sen_gear = 0 #current gear
        self.sen_speedX = 0 #cars longitudinal speed
        self.sen_laptime = 0 #current lap time, used to avoid premature restarts
        
        
        #### publication parameters ####
        self.ctrl_accel = 0 #desired accel signal [calculated for low constant speed]
        self.ctrl_brake = 0 #desired brake signal [fixed at 0]
        self.ctrl_steering = 0 #desired steering [calculated from trajectory]
        self.ctrl_gear = 0 #desired gear [calculated from look up table]
        self.needForAction_msg = BoolStamped() #message that indicates that end of a trajectory has been reached
        self.msg_restart = BoolStamped()
        self.msg_ctrl = TORCSCtrl() #message container

        #### calculation parameters and variables #### 
        self.param_kappa = 0.75/2 #value used to adjust desired heading to, experimentlly determined. 0.75 lead to small oscillation
        self.param_steerLock = np.deg2rad(21) #max. steerLock in torcs/data/cars/tbt1 is 21 deg
        self.trajectory_rot = vec4() #desired orientation from trajectory 
        self.sensors_topic = sensors_topic #message type used to identify whether the client node has been restarted
        self.race_start_time = rospy.Time.now()
        
        #### ensure all dependent nodes are up and running ####
        rospy.wait_for_message("/torcs_ros/notifications/NengoInitialized", Bool)
        rospy.wait_for_message(frame_topic, TFMessage) #wait until frame is published
        rospy.wait_for_message(sensors_topic, TORCSSensors) #also wait until sensors are published 
 
        #### publications ####
        self.pub_ctrl = rospy.Publisher(ctrl_topic, TORCSCtrl, queue_size=10) #publish control commands
        self.pub_needTrajectory = rospy.Publisher("/torcs_ros/notifications/ctrl_signal_action", BoolStamped, queue_size=1) #publish a control signal that indicates that a  new trajectory is needed
        self.pub_demandRestart = rospy.Publisher("/torcs_ros/notifications/demandRestart", BoolStamped, queue_size = 1)

        #### subscriptions ####
        self.sub_trajectory = rospy.Subscriber(trajectory_topic, Path, self.trajectory_callback) #subscribe to selected trajectory in world frame
        self.sub_sensors = rospy.Subscriber(sensors_topic, TORCSSensors, self.sensors_callback) #subscribe to sensor values for evaluation
        self.sub_frame = rospy.Subscriber(frame_topic, TFMessage, self.frame_callback, queue_size=1) #subscribe to frame transformations
        self.sub_speed = rospy.Subscriber(speed_topic, TwistStamped, self.speed_callback)
        

                


    #get trajectory
    def trajectory_callback(self, msg_trajectory):
        self.trajectory = msg_trajectory #get a new trajectory
        self.needForAction_msg.data = False #set flag to false everytime a new trajectory has been received
        
    #get sensor values
    def sensors_callback(self, msg_sensors):
        self.sen_angle = msg_sensors.angle #angle in rad
        self.sen_trackpos = msg_sensors.trackPos #normalized trackpos [-1, 1]
        self.sen_rpm = msg_sensors.rpm #rpm [no unit]
        self.sen_gear = msg_sensors.gear #current gear
        self.sen_laptime = msg_sensors.currentLapTime
        
    #get baselink frame position and orientation 
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
            # If new frame information has been published, new commands have to be calculated
            self.calculate_ctrl_commands() 

    def speed_callback(self, speed_msg):
            self.sen_speedX = speed_msg.twist.linear.x
           
    #calculate and publish control commands
    def calculate_ctrl_commands(self):
        if (self.sen_speedX < 30):
            self.ControlClassic();
            self.trajectory = Path() #reset as empty
            self.trajectory.poses = []
        elif (len(self.trajectory.poses) > 0): #ensure this callback does not happen before the first trajectory has been published
            
            [f_distToTraj, idx_poseHeading, f_distToEnd, self.needForAction_msg.data] = BaseLinkToTrajectory(
                    self.ros_trans.x, self.ros_trans.y, self.trajectory.poses) #Calculate baselink to trajectory information
            self.trajectory_rot.Set(self.trajectory.poses[idx_poseHeading]) #Convert msg type to vec4() quatenrion
            f_deltaHeading = self.ComputeAngleDifference(self.ros_rot, self.trajectory_rot) #difference in desired orientation to current baselink orientation
                
            self.ctrl_steering = (f_deltaHeading + f_distToTraj*self.param_kappa)/self.param_steerLock
            self.RPMandAccel();
            self.PublishCtrlMessage();

            

            if (f_distToEnd < 1.5): #check whether end of trajectory has been reached (within a margin)
                if(self.sen_trackpos <= 1):
                    self.needForAction_msg.data = True #indicate a new trajectory is needed
                else:
                    print("\033[31m Oh oh. Restart and end of trajectory collision. Not running nengo \033[0m")
                    self.needForAction_msg.data = False
            self.needForAction_msg.header.stamp = rospy.Time.now()
            self.pub_needTrajectory.publish(self.needForAction_msg) #send message whether new control is needed or not
            self.pub_ctrl.publish(self.msg_ctrl)            
            self.CheckForOutOfTrack() #restart server if vehicle is of track

        else: #no trajectory message has been received yet
            self.needForAction_msg.data = True #indicate a new trajectory is needed
            self.needForAction_msg.header.stamp = rospy.Time.now()
            self.pub_needTrajectory.publish(self.needForAction_msg) #send message that new trajectory is needed
            
    def ControlClassic(self):
         self.ctrl_steering = (self.sen_angle - self.sen_trackpos*self.param_kappa)/self.param_steerLock #control towards midline
         self.RPMandAccel()
         self.PublishCtrlMessage()
         
    def RPMandAccel(self):
         #limit acceleration signal to achieve low constant speeds
         if (self.sen_rpm < 4000 or self.sen_gear == 0): #allow high enough rpm if car is in neutral gear (start of race)
            self.ctrl_accel = 0.2
         else:
            self.ctrl_accel = 0
    
         self.ctrl_gear = SetGearFromLUT(self.sen_gear, self.sen_rpm) #gear lookup
            
    def PublishCtrlMessage(self):
        #add control data to message container
        self.msg_ctrl.accel = self.ctrl_accel
        self.msg_ctrl.steering = self.ctrl_steering
        self.msg_ctrl.brake = self.ctrl_brake #set to 0
        self.msg_ctrl.gear = self.ctrl_gear
        self.pub_ctrl.publish(self.msg_ctrl) #publish
        
    #Difference in angle [in rad] between vehicle orientation (baselink) to closest point on trajectory 
    #as we are working with a 2D assumption, the yaw value (angle around z) is the important value, as roll and pitch will be considered 0
    def ComputeAngleDifference(self, quad_baselink, quad_trajectory):
        [roll_bl, pitch_bl, yaw_bl] = tf.transformations.euler_from_quaternion(quad_baselink)
        [roll_tj, pitch_tj, yaw_tj] = tf.transformations.euler_from_quaternion(quad_trajectory)
        return yaw_tj-yaw_bl
        
    #function that restarts a race if the vehicle is of track
    def CheckForOutOfTrack(self):
        b_Restart = False
        if (abs(self.sen_trackpos) > 1): #if vehicle is off track or last point is , restart 
            if(self.sen_laptime > 5):
                b_Restart = True 
                print("\033[96mClient node is being restarted as vehicle was off track \033[0m") 

        elif (self.sen_speedX < 3): #if low speed, car is probably stuckF
#            dt = rospy.Time.now().secs - self.race_start_time.secs #calculate whether race has just started
            if(self.sen_laptime > 15):
#            if (dt > 15): #do not restart if race has just started
                b_Restart = True 
                print("\033[96mClient node is being restarted as vehicle seemed to be stuck \033[0m") 

        if b_Restart == True:
            self.pub_demandRestart.publish(self.msg_restart)
            self.trajectory = Path() #reset selected trajectory to be empty
            rospy.wait_for_message("/torcs_ros/notifications/isRestarted", BoolStamped)
            rospy.wait_for_message(self.sensors_topic, TORCSSensors) #wait for a message from client node to ensure it has restarted
            self.race_start_time = rospy.Time.now() #new race started, reinitialize time


if __name__ == "__main__":
    rospy.init_node("Trajectory_Follow_Control")
    Controller = FollowTrajectory()
    rospy.spin()