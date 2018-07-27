#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 19 10:18:44 2018

@author: bzorn
"""

import numpy as np
import rospy
import tf

## ros messages
from geometry_msgs.msg import TwistStamped, Vector3Stamped, PoseStamped, Point, PointStamped, TransformStamped, Quaternion
from visualization_msgs.msg import Marker
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Int8

import Trajectory_generation
import copy

import sys
import rospkg
cwd = rospkg.RosPack().get_path('torcs_ros_trajectory_gen')
sys.path.append(cwd[:-24] + "common")
cwd = cwd[:-24]
#print(cwd[:-24] + "common")
from bzVector import vec3, vec4 #2d and 3d vector definition; ERROR; roslaunch does not work with cwd
from bzReadTrajectoryParams import readTrajectoryParams

from bzGeometricFuncs import BaseLinkToTrajectory

#ROS node functionality
class TrajectoryPublish():
    def __init__(self, cwd, frame_topic = "/tf", action_topic="/torcs_ros/ctrl_signal_action", 
                 selector_topic = "/torcs_ros/TrajectorySelector"):
        #### variables ####
        self.time = rospy.Time.now() #to be used as reference time when performing transformations; not working yet
        self.tf_trans = [] #a transformations translatory values received from tr TransformListener
        self.tf_rot = [] #a transformations rotatory values received from tr TransformListener
        self.ros_trans = vec3() #a transformations translatory values received from ros subscription
        self.ros_rot = vec4() #a transformations rotatory values received from ros subscription
        self.newest_ros_transform = TransformStamped();  
        self.idx_TrajectorySelected  = 0
        self.b_TrajectoryNeeded = True 
        self.b_initHandshake = False
        self.b_initHandshakeSent = False
        #### parameters for trajectory generation ####
#        self.f_longitudinalDist = 25  #planning horizon in metre
#        self.f_lateralDist = 10 #lateral spread in width between terminal points in metre
#        self.n_amount = 10 #amount of trajectories per class
        [self.f_longitudinalDist, self.f_lateralDist, self.n_amount] = readTrajectoryParams(cwd)
        #### message definitions #####
        self.trajectoryBaselink_msgs = [] #trajectories in baselink coordinates (static) //depreceated
        self.trajectoryWorld_msgs = [] #trajectories in world coordinates (static) //depreceated
        self.path_msgs = [] #trajectories as paths in baselink coordinates 
        self.pathWorld_msgs = []
        self.selectedTrajectory_msg = Path() #currently selected trajectory to be followed
        self.selectedTrajectoryVis_msg = Path()
        
        #### trajectory parameters #### 
        self.n_selectedTrajectory = 0 #trajectory to be followed currently
        #initialization of static trajectory values
        self.y_vals, self.x_vals = [], [] #declaration needed as set_trajectories throws error otherwise
        [self.x_vals, self.y_vals] = Trajectory_generation.EgoTrajectories(self.f_lateralDist, 
            self.f_longitudinalDist, self.n_amount, False)
        
        self.set_trajectories() #initialize trajectories        

        #### publishers ####
         #list of publishers of trajectories in Path() format, one for each trajectory
        self.pub_allPaths = [] # //depreceated; list of publishers to visualize all possible trajectories
#        [self.pub_allPaths.append(rospy.Publisher("/torcs_ros/trajectory"+str(x), Path, queue_size=1)) for x in range(0, self.n_amount*2+1)] #
        self.pub_pathSelected = rospy.Publisher("/torcs_ros/trajectorySelected", Path, queue_size=1) #publisher for currently selected trajectory [in world frame]
        self.pub_pathSelectedVisual = rospy.Publisher("/torcs_ros/trajectorySelectedVis", Path, queue_size=1) #publisher for currently selected trajectory [in baselink frame]
        self.pub_handshake = rospy.Publisher("/torcs_ros/gen2selHandshake", Bool, queue_size=1)
        
        #### subscribers ####
        self.sub_frame = rospy.Subscriber(frame_topic, TFMessage, self.sub_frame_callback, queue_size=1) #a subscriber that manually subscribes to the published frames
#        self.sub_needForAction = rospy.Subscriber(action_topic, Bool, self.sub_needForAction_callback, queue_size=1) #receives message whether a new trajectory is needed
        self.sub_trajectorySelector = rospy.Subscriber(selector_topic, Int8, self.sub_trajectorySelector_callback, queue_size=1) #receives a trajectory index whenever a new trajectory is needed and selected
        
    def set_trajectories(self):
        ########## Path version of trajectories ##############
        
        
        self.path_msgs = [] #cleanup list for new calculation        
        ctime = rospy.Time.now() #assign all paths the same time
        #create one message for each trajectory
        for f in range(0, len(self.y_vals)):
            self.path_msgs.append(Path()) #push the Path() message type to list
            self.path_msgs[-1].header.stamp = ctime #assign same timestamp to all trajectories
            self.path_msgs[-1].header.frame_id = "base_link" #definition is in base_link to account for vehicles position and orientation
            #create poses for current path message 
            
            headings_cur = Trajectory_generation.ComputeHeadingsInRad(self.x_vals, self.y_vals[f])
            for ff in range (0, len(self.x_vals)):
                pose = PoseStamped() #PoseStamped needed for Path() message
                pose.header.stamp = ctime #Assign all poses same timestamp as path
                pose.header.frame_id = "base_link" #define poses in same frame as path
                pose.pose.position.x = self.x_vals[ff] #get x value from list
                pose.pose.position.y = self.y_vals[f][ff] #get y value from list
                pose.pose.position.z = 0 #set z value to 0 as we will ignore it
                #roll pitch and yaw needed for pose orientation, not filled as of yet
                roll = 0 #as trajectories are defined in 2D we can neglect this
                pitch = 0 #as trajectories are defined in 2D we can neglect this
                yaw = headings_cur[ff] #angle about z, orientation of current pose should be looking towards next post location

                quaternion = tf.transformations.quaternion_about_axis(-yaw-np.pi/2, (0, 0, 1))
                
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                
#                pose.pose.rotation = tf.transformations.quaternion_from_euler(roll, pitch, yaw) #convert rpy to quaternion
                self.path_msgs[-1].poses.append(pose)#append pose to current path message
        
        
        
        
    #mark a trajectory as active and visualize it in rviz with a different color by publishing it to another topic
    def mark_trajectory_i_as_active(self, trajectory):
        self.n_selectedTrajectory = trajectory #mark selected trajectory
        self.selectedTrajectory_msg = copy.copy(self.pathWorld_msgs[self.n_selectedTrajectory]) #Move message to selectedTrajectory publisher message
        self.pathWorld_msgs[self.n_selectedTrajectory].poses = [] #ensure old message does not interfere in visualization

    #depreceated; only here for debug purposes
    #manually transform all trajectories from baselink to world frame
    def transform_trajectories(self):
        self.pathWorld_msgs = [] #cleanup previous message
        self.ros_rot_mat = tf.transformations.quaternion_matrix(self.ros_rot) #calculate rotation matrix from quaternion
        for pathCounter in range(0, len(self.path_msgs)): #iterate over all trajectories
            self.pathWorld_msgs.append(copy.deepcopy(self.path_msgs[pathCounter])) #create a new message with a trajectory as content
#            self.pathWorld_msgs.append(copy.copy(self.path_msgs[pathCounter])) #create a new message with a trajectory as content
            self.pathWorld_msgs[-1].header.frame_id = 'world' #set that trajectories frame to world instead
            self.pathWorld_msgs[-1].header.stamp = self.time
            #transform trajectory poses to current base_link position
            for pose in self.pathWorld_msgs[-1].poses:
                pose.header.frame_id = 'world' #set PoseStamped Header
                pose.header.stamp = self.time
                #rotate pose by quaternion with a rotation matrix
                #results are same when comparing to rotating v1 by quaternion q1 with q1*v1*(q1^*)
                position_homogenous = np.asarray([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 1]) #make a homogenous position vector, needed as rot. matrix is 4x4
                position_rotated = np.matmul(self.ros_rot_mat, position_homogenous) #implementation with matrix is neglible faster than quaternion implementation
                #translate rotated postion to baselink origin    
                pose.pose.position.x = position_rotated[0] + self.ros_trans.x 
                pose.pose.position.y = position_rotated[1] + self.ros_trans.y
                pose.pose.position.z = position_rotated[2] + self.ros_trans.z
                quaternion_base = np.asarray([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, 
                                         pose.pose.orientation.w]) #numpy array for orientation quaternion
                quaternion_rotated = tf.transformations.quaternion_multiply(quaternion_base, self.ros_rot) #rotate pose by baselink orientation
                #write new data to Pose Message
                pose.pose.orientation.x = quaternion_rotated[0]
                pose.pose.orientation.y = quaternion_rotated[1]
                pose.pose.orientation.z = quaternion_rotated[2]
                pose.pose.orientation.w = quaternion_rotated[3]


    #manually transform a certain trajectories from baselink to world frame
    def transform_one_trajectories(self, idx):
        self.pathWorld_msgs = [] #cleanup previous message
        self.ros_rot_mat = tf.transformations.quaternion_matrix(self.ros_rot) #calculate rotation matrix from quaternion
        self.pathWorld_msgs.append(copy.deepcopy(self.path_msgs[idx])) #create a new message with a trajectory as content
#            self.pathWorld_msgs.append(copy.copy(self.path_msgs[pathCounter])) #create a new message with a trajectory as content
        self.pathWorld_msgs[-1].header.frame_id = 'world' #set that trajectories frame to world instead
        self.pathWorld_msgs[-1].header.stamp = self.time
        #transform trajectory poses to current base_link position
        for pose in self.pathWorld_msgs[-1].poses:
            pose.header.frame_id = 'world' #set PoseStamped Header
            pose.header.stamp = self.time
            #rotate pose by quaternion with a rotation matrix
            #results are same when comparing to rotating v1 by quaternion q1 with q1*v1*(q1^*)
            position_homogenous = np.asarray([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 1]) #make a homogenous position vector, needed as rot. matrix is 4x4
            position_rotated = np.matmul(self.ros_rot_mat, position_homogenous) #implementation with matrix is neglible faster than quaternion implementation
            #translate rotated postion to baselink origin    
            pose.pose.position.x = position_rotated[0] + self.ros_trans.x 
            pose.pose.position.y = position_rotated[1] + self.ros_trans.y
            pose.pose.position.z = position_rotated[2] + self.ros_trans.z
            quaternion_base = np.asarray([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, 
                                     pose.pose.orientation.w]) #numpy array for orientation quaternion
            quaternion_rotated = tf.transformations.quaternion_multiply(quaternion_base, self.ros_rot) #rotate pose by baselink orientation
            #write new data to Pose Message
            pose.pose.orientation.x = quaternion_rotated[0]
            pose.pose.orientation.y = quaternion_rotated[1]
            pose.pose.orientation.z = quaternion_rotated[2]
            pose.pose.orientation.w = quaternion_rotated[3]
            
    #callback function that gets the baselink transformation and publishes the transformed trajectories in the world frame          
    def sub_frame_callback(self, msg_frame):
        frame_idx = 0 #index to identify the proper transformation if there is more than the world and base_link frames avalaible
        b_frameHasBeenFound = False #flag to show if transformation has been found
        #identify transformation
        for frame in msg_frame.transforms:
            if frame.header.frame_id == "world" and frame.child_frame_id == "base_link":
                b_frameHasBeenFound = True #set flag that a valid transformation has been published
                break;
            ++frame_idx
#        
        if b_frameHasBeenFound == True: #ensure there is data to use
            self.newest_ros_transform = msg_frame.transforms[frame_idx] #get transform from message
            self.time = msg_frame.transforms[frame_idx].header.stamp #get time from message
            self.ros_trans.Set(msg_frame.transforms[frame_idx]) #Get translation from published ros message
            self.ros_rot.Set(msg_frame.transforms[frame_idx]) #Get rotation from published ros message
          
            #check whether a new trajectory has to be published
            if self.b_TrajectoryNeeded == True:
                self.selectAndPublishTrajectory()
                self.b_TrajectoryNeeded = False

            
            self.transformAndPublishVisualization() #always publish trajectory in baselink frame, as the relative movement has to be compensated for

        
        
    #callback function that identifies whether a new trajectory has to be selected and published
#    def sub_needForAction_callback(self, msg_action):
    def sub_trajectorySelector_callback(self, msg_selector):
        self.idx_TrajectorySelected = msg_selector.data
        self.b_TrajectoryNeeded = True
        self.b_initHandshake = True #can be marked as true as soon as the first trajectory has been received
        
    def selectAndPublishTrajectory(self):
#        self.transform_one_trajectories(np.random.randint(0, self.n_amount*2+1)) #transform trajectories to world coordinates
        self.transform_one_trajectories(self.idx_TrajectorySelected) #transform trajectories to world coordinates

        self.mark_trajectory_i_as_active(0) #set a chosen trajectory as active

#        #publish all trajectories apart from selected one (message is empty)
#        for path_msg, path_pub in zip (self.pathWorld_msgs, self.pub_allPaths):
#            if path_msg is not self.path_msgs[self.n_selectedTrajectory]:
#                path_pub.publish(path_msg)

        self.pub_pathSelected.publish(self.selectedTrajectory_msg)


    #Function that (manually) transforms the trajectory in world coordinates back to the baselink
    #Transform from base_link to world was: x' = Rx+t [R is rotation matrix and t is translation vector]
    #Inverse transform therefore is (R^-1)x'-(R^-1)t=x
    def transformAndPublishVisualization(self):
        self.ros_rot_mat = tf.transformations.quaternion_matrix(self.ros_rot) #get R
        ros_rot_mat_inv = tf.transformations.inverse_matrix(self.ros_rot_mat) #calculate inverse R^-1
        self.selectedTrajectoryVis_msg = copy.deepcopy(self.selectedTrajectory_msg) #copy trajectory in world coordinates
        self.selectedTrajectoryVis_msg.header.stamp = self.time #set stamp to time of last received transform 
        self.selectedTrajectoryVis_msg.header.frame_id = 'base_link' #change frame_id to baselink
        for pose in self.selectedTrajectoryVis_msg.poses: #transform all pose positions. neglect orientations
            pose.header.frame_id = 'base_link' #set pose to new frame
            pose.header.stamp = self.time #adjust time
            #perform inverse transform
            position_homogenous = np.asarray([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 1]) #make a homogenous position vector, needed as rot. matrix is 4x4
            position_rotated = np.matmul(ros_rot_mat_inv, position_homogenous) #implementation with matrix is neglible faster than quaternion implementation    
            translation_inv = np.matmul(ros_rot_mat_inv, [self.ros_trans.x, self.ros_trans.y, self.ros_trans.z, 1]) #calculate inverse translation
            #apply inverse translation
            pose.pose.position.x = position_rotated[0] - translation_inv[0]
            pose.pose.position.y = position_rotated[1] - translation_inv[1]
            pose.pose.position.z = position_rotated[2] - translation_inv[2]
        self.pub_pathSelectedVisual.publish(self.selectedTrajectoryVis_msg)
    
    def initHandshake(self):
        if (self.b_initHandshakeSent == False):
            self.b_initHandshakeSent = True
            handshake_message = Bool()
            handshake_message.data = self.b_initHandshake
            self.pub_handshake.publish(self.b_initHandshake)
        
if __name__ == '__main__':
    rospy.init_node('Trajectory_Publisher')
    rospy.wait_for_message("/tf", TFMessage)
    trajectoryPublisher_ = TrajectoryPublish(cwd)
    while not rospy.is_shutdown():
        rospy.sleep(5)
        trajectoryPublisher_.initHandshake()

