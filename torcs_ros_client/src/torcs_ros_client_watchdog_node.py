#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 26 12:59:36 2018

@author: bzorn
"""

import os
import rospy
import numpy as np

from torcs_msgs.msg import TORCSSensors



class Watchdog():
    def __init__(self, sensors_topic = "/torcs_ros/sensors_state"):
        
        self.last_callback_time = rospy.Time.now()
            
        self.sub_sensors = rospy.Subscriber(sensors_topic, TORCSSensors, callback = self.callback)
        

    def callback(self, msg):
        self.last_callback_time = rospy.Time.now()

    #the restart will only work if the game is waiting for the scr_server as it will not connect otherwise
    def is_client_alive(self):
        str_nodes = os.popen("rosnode list").readlines() #returns rosnode list as list where every line of output is one list element
        #check whether client node is alive by checking for inclusion in output (any() returns true if one of the elemnts is true)
        if not (np.array(['/torcs_ros/torcs_ros_client_node' in string for string in str_nodes]).any()): 
            print("Watchdog has identified that client node is not active. Waiting for 30 seconds before restarting")
            n_Counter = 0 #time indicator
            while(True):
                rospy.sleep(1) #wait for 1 second
                n_Counter += 1 #count to 30 seconds
                str_nodes = os.popen("rosnode list").readlines() #returns rosnode list as list where every line of output is one list element
                #abort if client node is alive after all
                if  (np.array(['/torcs_ros/torcs_ros_client_node' in string for string in str_nodes]).any()): 
                    print("Client node is alive after all. Not going to restart")
                    break;
                #restart if client node has been dead for 30 seconds
                if (n_Counter == 30):
                    print("\033[96mWatchdog says: Client node seems to have died and not been restarted. Will try starting it again. \033[97m") 
                    rospy.sleep(1) #give the system enough time to have killed of node
                    #  os.system("roslaunch torcs_ros_client torcs_ros_client_ns.xml") #relaunch client node with roslaunch command when not in namespace yet (if called manually from console)
                    os.system("roslaunch torcs_ros_client torcs_ros_client.xml")
                    break;

    #a function that checks whether the client is connected to the server by comparing the current time
    #to the time when the last sensor message was received.
    #if the client is up but no sensor messages are received it is probable that the client is not connected
    def has_client_connected(self):
        str_nodes = os.popen("rosnode list").readlines() #returns rosnode list as list where every line of output is one list element
        #see if client node is in rosnode list and therefore alive
        if (np.array(['/torcs_ros/torcs_ros_client_node' in string for string in str_nodes]).any()): 
            td = rospy.Time.now().secs - self.last_callback_time.secs #get time between now and last received sensor message
            if (td > 30): #kill and restart node if time is above 30 seconds
                os.system("rosnode kill /torcs_ros/torcs_ros_client_node") #kills client node with terminal command
                print("\033[96mWatchdog says: Client node is alive but seems to not be connected to torcs. Will try restarting it.  \033[97m") 
                rospy.sleep(1) #give the system enough time to have killed of node
#               os.system("roslaunch torcs_ros_client torcs_ros_client_ns.xml") #relaunch client node with roslaunch command when not in namespace yet (if called manually from console)
                os.system("roslaunch torcs_ros_client torcs_ros_client.xml") #relaunch client node with roslaunch command when in namespace (if launched with bringup .launch file)



if __name__ == "__main__":
    rospy.init_node("torcs_ros_client_watchdog")
    watchdog = Watchdog()
    while not rospy.is_shutdown(): 
        watchdog.is_client_alive() #check whether client is dead
        watchdog.has_client_connected() #check whether client is connected to torcs server
        rospy.sleep(1)
        