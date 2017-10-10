# -*- coding: utf-8 -*-
"""
Created on Wed Jun 28 09:04:17 2017

@author: Zorn, Ronecker
"""

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue May 30 17:18:42 2017

@author: ben
"""

import numpy as np
import rospy
import os
import sys
import h5py
import datetime
import cv2
import message_filters

from torcs_msgs.msg import TORCSSensors
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from torcs_msgs.msg import TORCSCtrl
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import keras
from keras.models import load_model
import tensorflow as tf





class Controler:
    def __init__(self, image_topic='torcs_ros/pov_image',control_topic='/torcs_ros/ctrl_cmd', sensors_topic='/torcs_ros/sensors_state'):
        self.model_path = '/home/max/Schreibtisch/weights_epoch_99.h5' #load neural net at path
        self.bridge = CvBridge() #instance to convert image data
        self.net = self.load_net() #load net
        self.control_msg = TORCSCtrl()
        self.pub_control = rospy.Publisher(control_topic, TORCSCtrl, queue_size=50)
        
        #Subscriptions
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.torcs_sensors_sub = rospy.Subscriber(sensors_topic, TORCSSensors, self.sensors_callback) # Sensor subscriptions
        self.control_sub = rospy.Subscriber(control_topic, TORCSCtrl, self.control_callback)
      	self.gear = 0
      	self.rpm = 0

        self.data ={}
        self.data['image'] = [] #parsed image
        
    def load_net(self):
        net = load_model(self.model_path)
        self.graph = tf.get_default_graph() #graph instance needed as prediction is made in different function
        return net
        
        
    def passed_time(self): #passed time since last publish
        try:
            self.t2 = self.t1
        except:
            self.t2 = 0
    
        self.t1 = rospy.get_time()
        print(self.t2)
        passed_t = self.t1-self.t2
        print("Time since last publish: " + str(passed_t))

    def image_callback(self, image_msg):
        image_message = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough") #rows 640, cols 480, channels bgr 3
       # print("Red 640 480:" + str(image_message[-1][-1][2])) # (Example Pixel of 640x480xbgr)
        self.data['image'].append(np.true_divide(image_message,255))#limit between [0,1]
        self.data['image'][-1] = np.expand_dims(self.data['image'][-1], axis = 0) #reshape for prediction
        self.make_prediction() #predict
        self.control_publish() #publish
        self.passed_time() #time message
        
    def make_prediction(self):
        self.graph #needed as net is loaded in different function
        with self.graph.as_default():
            self.prediction = self.net.predict(self.data['image'][-1], batch_size =1, verbose = 0) #make prediction
        del(self.data['image'])
        print(self.prediction)
        self.data['image'] = []      
#        self.accel = self.prediction[0][0] #if three outputs
#        self.brake = self.prediction[0][1] #if three outputs
        self.steering = (self.prediction[0][-1])*2-1
        
    def sensors_callback(self, sensors_msg):
      	self.rpm = sensors_msg.rpm;
      	
    def control_callback(self,control_msg):

    	self.gear = control_msg.gear

     #Definiton of current gear, identical to preimplemnted controller
    def get_gear(self):
        gearUp = [5000,6000,6000,6500,7000,0]
        gearDown = [0,2500,3000,3000,3500,3500]
    	if(self.gear < 1):
    		return 1
    	
    	if(self.gear <6 and self.rpm > gearUp[(self.gear-1)]):
    		return self.gear +1
    	else:
    		if (self.gear > 1 and self.rpm < gearDown[(self.gear-1)]):
    			return self.gear -1
    		else:
    			return self.gear

    	
        
    #publish data
    def control_publish(self):
        self.control_msg.accel = 0.4
        self.control_msg.brake = 0
        self.control_msg.steering = self.steering
        self.control_msg.gear = self.get_gear()
        self.pub_control.publish(self.control_msg)

        
        
    
    

if __name__ == '__main__':
    print("Starting up")
    rospy.init_node('NN_Control_Node')
    NN_control = Controler()
    
    rospy.spin()
        
