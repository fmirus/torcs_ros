#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue May 30 17:18:42 2017

@author: Zorn, Ronecker
"""

import numpy as np
import rospy
import os
import sys
import h5py
import datetime
import cv2
import message_filters
import yaml

from torcs_msgs.msg import TORCSSensors
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from torcs_msgs.msg import TORCSCtrl
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String


data = {}
data['image']   = []
data['time'] = []
class DataLogger:
    def __init__(self, logfile_name='log_1.h5', sensors_topic='torcs_ros/sensors_state',image_topic='torcs_ros/pov_image',control_topic='/torcs_ros/ctrl_state', scan_track_topic='torcs_ros/scan_track', log_at_start=True):
        
        
        self.b_logging = log_at_start #only log if set to true
        self.readYaml()
        self.find_dir() #find current working directory
        self.logfile_name = logfile_name #set file name
        self.log_file = self.logfile_path #rename
        self.bridge = CvBridge() #OpenCV instance that can be used to convert image message
        self.create_data_trigger() #Initialize Logging .h5 file
        #### Publishers if Data should be republished ####
        self.pub_image = rospy.Publisher('torcs_ros/synched/pov_image', Image, queue_size=50)
        self.pub_sensors = rospy.Publisher('torcs_ros/synched/sensors_state', TORCSSensors, queue_size=50)
        self.pub_control = rospy.Publisher('torcs_ros/synched/ctrl_state', TORCSCtrl, queue_size=50)
        self.pub_scan = rospy.Publisher('torcs_ros/synched/scan_track', LaserScan, queue_size=50)
        ##################################################
        
        #### Subscriptions ####
        self.image_sub = message_filters.Subscriber(image_topic, Image)
        self.sensors_sub = message_filters.Subscriber(sensors_topic, TORCSSensors)
        self.control_sub = message_filters.Subscriber(control_topic, TORCSCtrl)
        self.scan_sub = message_filters.Subscriber(scan_track_topic, LaserScan)
        #######################
        
        #Function that synchronizes all input messages with same time stamp (within slop factor varicance)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.sensors_sub, self.control_sub, self.scan_sub], 10, 0.2)
        
        #### callback functions ####
        self.synchronizer.registerCallback(self.sensors_callback)
        self.synchronizer.registerCallback(self.control_callback)
        self.synchronizer.registerCallback(self.scan_track_callback)
        self.synchronizer.registerCallback(self.image_callback) #do last as it takes the most time
        self.synchronizer.registerCallback(self.passed_time)
        ############################
        
        #### arrays/dictionaries for data ####
        self.data ={}
        self.data['angle'] = []
        self.data['track_pos'] = [] #used for eval; has to stay on track; between -1 and 1
        self.data['image'] = [] #parsed image
        self.data['image2'] = [] 
        self.data['height'] = []
        self.data['width'] = []
        self.data['damage'] = [] #damage of vehicle, used for evaluation
        self.data['distFromStart'] = [] #Distance from start on track, used for eval
        self.data['lastLapTime'] = [] #Last lap time, maybe used for eval. Otherwise use amount of data logged
        self.data['currentLapTime'] = [] 
        self.data['lapTimes'] = []
        self.data['evalLapTime'] = [] #used for evaluation
#        self.data['evalLapTime'].append(0)
        self.data['evalLapTimes'] = [] #used for evaluation
        self.data['lapTimes'].append(0)
        self.data['evalLapTimes'].append(0)
        self.data['ctrl_Accel'] = [] #Gas Pedal (0 no gas, 1 full gas)
#        self.data['ctrl_Accel2'] = [] #Gas Pedal (0 no gas, 1 full gas)
        self.data['ctrl_Brake'] = [] #Brake Pedal (0 no gas, 1 full gas)
        self.data['ctrl_Stear'] = [] #Stear Angle (-1 full left, 1 full right)
        self.data['ctrl_Clutch'] = [] #Clutch (-)
        self.data['ctrl_Gear'] = [] #Gear, -1 backwards, 0 idle, up to 6
        self.data['scan_track'] = []
        
        #help variables
        self.c1 = 0 
        self.c2 = 0 
        self.e1 = 0
        self.l1 = 0
        self.l2 = 0
        
        
    # Define Logfile Path
    def find_dir(self):
        self.logfile_path = os.getcwd() #current working directory
        counter = 0
        for f in range(len(self.logfile_path)):#hardcoding for if cwd is within catkin workspace 
            try:
                if self.logfile_path[f:f+3] == "src":
                    self.logfile_path = self.logfile_path[0:f-1] + "/training_data"
            except:
                pass
        if not os.path.exists(self.logfile_path): #make folder
            os.makedirs(self.logfile_path)
            print ("Had to create new folder for data at: " + self.logfile_path)
        now = datetime.datetime.now() #current time
        self.filename = "logged_data-"
        self.extension = str(now.year) + "-" + str(now.month) + "-" + str(now.day) + "_" + str(now.hour) + ":" + str(now.minute) + "-" + self.extension
        self.logfile_path = self.logfile_path + "/" + self.filename + self.extension + ".h5" #file name with time
    
        
    def readYaml(self):
        yaml_path = os.getcwd() + "/data_logger_parameters.yaml"

        with open(yaml_path, 'r') as ymlfile:
            params = yaml.load(ymlfile)
	self.sleep_rate = params['sleep_rate'] #variable used to limit data to every x seconds
	self.extension = params['extension'] #define user prefix for file name
	self.evalFactor = params['evalFactor'] #factor used for eval
	self.sample_buffer = params['sample_buffer']; #cache size before storing (currently only "1" debugged)

    #sensor callback
    def sensors_callback(self, image_msg, sensors_msg, control_msg, scan_msg):
          if self.b_logging == True: #check whether logging is set to true
              
              #append messag to dictionary
              self.pub_sensors.publish(sensors_msg)
              self.data['angle'].append(sensors_msg.angle)
              self.data['track_pos'].append(sensors_msg.trackPos)
              self.data['damage'].append(sensors_msg.damage)
              self.data['distFromStart'].append(sensors_msg.distFromStart)
              self.data['lastLapTime'].append(sensors_msg.lastLapTime)
              self.data['currentLapTime'].append(sensors_msg.currentLapTime)
              #handling of elapsed time since last sample (used for eval)
              try:
                  if self.data['currentLapTime'][-1] > self.c1:
                      elapsedTime = self.data['currentLapTime'][-1]-self.c1
                  else:
                      elapsedTime = self.data['currentLapTime'][-1]
                  # adjust elapsed time if car is off track
                  if abs(self.data['track_pos'][-1]) > 1: #car is outside of track
                     print("Out of track")
                     elapsedTime = elapsedTime*abs(self.data['track_pos'][-1])*self.evalFactor;
                  if self.data['currentLapTime'][-1] < 0: #race hasn't started yet
                          self.data['evalLapTime'].append(0)
                  else:
                      if self.data['currentLapTime'][-1] > 0 and self.c1 < 0: #race has started with this sample
                          elapsedTime = self.data['currentLapTime'][-1];
                      self.data['evalLapTime'].append(elapsedTime+self.e1)
              except:
                  self.data['evalLapTime'].append(0)
              try:
                 if self.data['currentLapTime'][-1] > 1:
                     self.data['lapTimes'][-1] = self.data['currentLapTime'][-1]
                 else:
                     self.data['lapTimes'][-1] = self.c1 #append current lap time to last lap time
                 self.data['evalLapTimes'][-1] = ((self.data['evalLapTime'][-1])) #append eval lap time to eval lap times
                 if  self.data['currentLapTime'][-1] > 0 and self.data['currentLapTime'][-1] < self.c1: #check whether new round has started
                     self.data['lapTimes'].append(0)
                     self.data['evalLapTimes'].append(0)
                     self.data['evalLapTime'][-1] = self.data['currentLapTime'][-1]
              except:
                 pass
              print("saved sensors")
              self.write_sensors_trigger()


    
    def control_callback(self, image_msg, sensors_msg, control_msg, scan_msg):
        if self.b_logging == True: #check whether logging is set to true
            self.pub_control.publish(control_msg)
            #append control message
            self.data['ctrl_Brake'].append(control_msg.brake)
            self.data['ctrl_Stear'].append(control_msg.steering)
            self.data['ctrl_Clutch'].append(control_msg.clutch)
            self.data['ctrl_Gear'].append(control_msg.gear)
            self.data['ctrl_Accel'].append(control_msg.accel)
            self.write_ctrl_trigger()
            print("saved control")    
        
        
    def scan_track_callback(self, image_msg, sensors_msg, control_msg, scan_msg):
        if self.b_logging == True: #check whether logging is set to true
            self.pub_scan.publish(scan_msg)
            #append lidar message
            self.data['scan_track'].append(scan_msg.ranges)
            self.write_scan_trigger()
            print("saved scan")

    # Image Callback. Append Image string to image dictionary
    def image_callback(self, image_msg, sensors_msg, control_msg, scan_msg):
        if self.b_logging == True:
            self.pub_image.publish(image_msg)
            self.data['image'].append(np.string_(image_msg.data))
            self.write_img_trigger() #Call the writing of image data to log
            print("saved img")
            if self.sleep_rate is not 0: #Sleep for sleep_rate
                print("You have chosen to limit the data. Sleeping for " + str(self.sleep_rate) + " seconds")
                rospy.sleep(self.sleep_rate)
        
    # Print Message of time passed between saved samples
    def passed_time(self, image_msg, sensors_msg, control_msg, scan_msg):
        try:
            self.t2 = self.t1 
        except: 
            self.t2 = 0
    
        self.t1 = rospy.get_time()
        print(self.t2)
        passed_t = self.t1-self.t2
        print("Time since last save: " + str(passed_t))    
    
    #%% Write triggers are all structured the same:
    # Resize hdf dataset and append previous buffer to each set
        
    def write_ctrl_trigger(self):
        hf = h5py.File(self.log_file, 'r+')
        hf['ctrl_Accel'].resize((hf['ctrl_Accel'].shape[0]+self.sample_buffer,))
        hf['ctrl_Brake'].resize((hf['ctrl_Brake'].shape[0]+self.sample_buffer,))
        hf['ctrl_Stear'].resize((hf['ctrl_Stear'].shape[0]+self.sample_buffer,))
#        hf['ctrl_Clutch'].resize((hf['ctrl_Clutch'].shape[0]+self.sample_buffer))
#        hf['ctrl_Gear'].resize((hf['ctrl_Gear'].shape[0]+self.sample_buffer))
        hf['ctrl_Accel'][-self.sample_buffer:] = self.data['ctrl_Accel']
        hf['ctrl_Brake'][-self.sample_buffer:] = self.data['ctrl_Brake']
        hf['ctrl_Stear'][-self.sample_buffer:] = self.data['ctrl_Stear']
#        hf['ctrl_Clutch'][-self.sample_buffer:] = self.data['ctrl_Clutch']
#        hf['ctrl_Gear'][-self.sample_buffer:] = self.data['ctrl_Gear']
        del(self.data['ctrl_Accel'])
        self.data['ctrl_Accel'] = []
        del(self.data['ctrl_Brake'])
        self.data['ctrl_Brake'] = []
        del(self.data['ctrl_Stear'])
        self.data['ctrl_Stear'] = []
#        del(self.data['ctrl_Clutch'])
#        self.data['ctrl_Clutch'] = []
#        del(self.data['ctrl_Gear'])
#        self.data['ctrl_Gear'] = []
        hf.close()
        return
    
    
    def write_img_trigger(self):
        hf = h5py.File(self.log_file, 'r+') 
        hf['img_image'].resize((hf['img_image'].shape[0]+self.sample_buffer,))
#        hf['img_image2'].resize((hf['img_image'].shape[0]+self.sample_buffer,480,640,3))
        
        hf['img_image'][-self.sample_buffer:] = self.data['image'][0]
#        hf['img_image2'][-self.sample_buffer:] = self.data['image2'][0]
        del(self.data['image'])
        self.data['image'] = [] 
#        del(self.data['image2'])
        self.data['image2'] = []
        hf.close()
        

        return
    
    
    
    def write_sensors_trigger(self):
      #handling of help variables
      self.l2 = self.l1
      self.l1 = self.data['lastLapTime'][-1]
      self.c2 = self.c1
      self.c1 = self.data['currentLapTime'][-1]
      self.e1 = self.data['evalLapTime'][-1]

      hf = h5py.File(self.log_file, 'r+')  
      hf['sen_angle'].resize((hf['sen_angle'].shape[0]+self.sample_buffer,))
      hf['sen_currentLapTime'].resize((hf['sen_currentLapTime'].shape[0]+self.sample_buffer,))
      hf['eval_evalLapTime'].resize((hf['eval_evalLapTime'].shape[0]+self.sample_buffer,))
      hf['sen_damage'].resize((hf['sen_damage'].shape[0]+self.sample_buffer,))
      hf['sen_distFromStart'].resize((hf['sen_distFromStart'].shape[0]+self.sample_buffer,))  
      if len(self.data['lapTimes']) is not hf['sen_lapTimes'].shape[0]:
          hf['sen_lapTimes'].resize((hf['sen_lapTimes'].shape[0]+self.sample_buffer,))
      hf['sen_lapTimes'][-self.sample_buffer:] = self.data['lapTimes'][-1]
      if len(self.data['evalLapTimes']) is not hf['eval_evalLapTimes'].shape[0]:
          hf['eval_evalLapTimes'].resize((hf['eval_evalLapTimes'].shape[0]+self.sample_buffer,))
      hf['eval_evalLapTimes'][-self.sample_buffer:] = self.data['evalLapTimes'][-1]
      hf['sen_track_pos'].resize((hf['sen_track_pos'].shape[0]+self.sample_buffer,))
      hf['sen_angle'][-self.sample_buffer:] = self.data['angle'][0]
      del(self.data['angle'])
      self.data['angle'] = [] 
      hf['sen_currentLapTime'][-self.sample_buffer:] = self.data['currentLapTime'][0]
      del(self.data['currentLapTime'])
      self.data['currentLapTime'] = [] 
      hf['sen_damage'][-self.sample_buffer:] = self.data['damage'][0]
      del(self.data['damage'])
      self.data['damage'] = [] 
      hf['sen_distFromStart'][-self.sample_buffer:] = self.data['distFromStart'][0]
      del(self.data['distFromStart'])
      self.data['distFromStart'] = [] 
#      hf['sen_scan_track'][-self.sample_buffer:] = self.data['scan_track'][0]
#      del(self.data['scan_track'])
#      self.data['scan_track'] = []       
      hf['sen_track_pos'][-self.sample_buffer:] = self.data['track_pos'][0]
      del(self.data['track_pos'])
      self.data['track_pos'] = []       
      hf['eval_evalLapTime'][-self.sample_buffer:] = self.data['evalLapTime'][0]
      del(self.data['evalLapTime'])
      self.data['evalLapTime'] = []     
      hf.close()
      return
  
    
    
    def write_scan_trigger(self):
        hf = h5py.File(self.log_file, 'r+')
        hf['sen_scan_track'].resize((hf['sen_scan_track'].shape[0]+self.sample_buffer,19))
        hf['sen_scan_track'][-self.sample_buffer:] = self.data['scan_track'][0]
        del(self.data['scan_track'])
        self.data['scan_track'] = []   
        hf.close()
        return
  
    
    
    def create_data_trigger(self):
        
        print("Logging started to:" + self.logfile_path)
        print("Image Data Saved as String. Run Picture Converter afterwards")
        success = True
        message = 'Successfully logged data to file ' + self.log_file
        try:

          # safe data to h5 file

          with h5py.File(self.log_file, 'w') as hf:
           #control data
            hf.create_dataset('ctrl_Accel', (0,), maxshape=(None,), compression="gzip", compression_opts=9)
            hf.create_dataset('ctrl_Brake', (0,), maxshape=(None,),compression="gzip", compression_opts=9)
            hf.create_dataset('ctrl_Stear', (0,), maxshape=(None,),compression="gzip", compression_opts=9)
            hf.create_dataset('ctrl_Clutch', (0,), maxshape=(None,),compression="gzip", compression_opts=9)
            hf.create_dataset('ctrl_Gear', (0,), maxshape=(None,),compression="gzip", compression_opts=9)
#            
#            #eval data
            hf.create_dataset('eval_evalLapTime', (0,), maxshape=(None,),compression="gzip", compression_opts=9)
            hf.create_dataset('eval_evalLapTimes',(0,), maxshape=(None,), compression="gzip", compression_opts=9)
#            
#            #image data
            hf.create_dataset('img_image', (0,), maxshape=(None,),compression="gzip", compression_opts=9, dtype="S921600")
#            hf.create_dataset('img_height', (0,), maxshape=(None,),compression="gzip", compression_opts=9)
#            hf.create_dataset('img_width', (0,), maxshape=(None,),compression="gzip", compression_opts=9)
#            hf.create_dataset('img_encoding', data=self.data['img_enc'], compression="gzip", compression_opts=9)
#            
#            #sensory data
            hf.create_dataset('sen_angle', (0,), maxshape=(None,),compression="gzip", compression_opts=9)
            hf.create_dataset('sen_currentLapTime',(0,), maxshape=(None,), compression="gzip", compression_opts=9)
            hf.create_dataset('sen_damage', (0,), maxshape=(None,),compression="gzip", compression_opts=9)
            hf.create_dataset('sen_distFromStart', (0,), maxshape=(None,),compression="gzip", compression_opts=9)
            hf.create_dataset('sen_lapTimes',(0,), maxshape=(None,), compression="gzip", compression_opts=9)
            hf.create_dataset('sen_scan_track', (0,19), maxshape=(None,19),compression="gzip", compression_opts=9)
            hf.create_dataset('sen_track_pos', (0,), maxshape=(None,),compression="gzip", compression_opts=9)
            hf.close()
        
        except:
          success = False
          message = 'Something went wrong during logging data to file ' + self.log_file
          print("wrong")
    
        return [success, message]
 

if __name__ == '__main__':

    rospy.init_node('Synchronized_DataLogger') #initalize code
    my_logger = DataLogger() #create instance
    
    rospy.spin()
#        
