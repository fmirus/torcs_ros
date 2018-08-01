#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 26 12:59:36 2018

@author: bzorn
"""

import os
import rospy
import numpy as np
import subprocess 

from torcs_msgs.msg import TORCSSensors
from rosgraph_msgs.msg import Log

import sys
import rospkg
cwd = rospkg.RosPack().get_path('torcs_ros_client')#trajectory_gen')
sys.path.append(cwd[:-16] + "common")
cwd = cwd[:-16]

from bzRestartTORCSraceGUI import RestartTORCSRace
from std_msgs.msg import Bool
class Watchdog():
    def __init__(self, sensors_topic = "/torcs_ros/sensors_state"):
        
        self.last_callback_time = rospy.Time.now()
        self.log_callback_time_connection = rospy.Time.now()
        self.log_callback_time_pause = rospy.Time.now()
        self.b_HasBeenLaunchedOnce = False

        self.n_counterConnection = 0
        self.n_counterPause = 0
        self.n_counterComp = 20

        self.pub_demandPause = rospy.Publisher("/torcs_ros/demandPause", Bool,queue_size = 1)
            
        self.sub_sensors = rospy.Subscriber(sensors_topic, TORCSSensors, callback = self.callback)
        self.sub_log = rospy.Subscriber("/rosout", Log, callback =self.log_callback)
        

    def callback(self, msg):
        self.last_callback_time = rospy.Time.now()
        self.b_HasBeenLaunchedOnce = True
        self.n_counterComp = 5
        
    def log_callback(self, msg):
        if(self.b_HasBeenLaunchedOnce): #avoid clicking before first run
#            if ("torcs_ros_client_node.cpp" in msg.file):
#                pass
#                if ("Not connected to server yet!!" in msg.msg): #theoretically it should be enough to check for this
#                    dt = rospy.Time.now() - self.log_callback_time_connection 
#                    self.log_callback_time_connection = rospy.Time.now()
#                    if(dt.secs < 3):
#                        self.n_counterConnection += 1
#                        if(self.n_counterConnection >= self.n_counterComp):
#                            print("\033[96mWatchdog is restarting TORCS via the GUI and restarting client node \033[0m")
#                            os.system("rosnode kill /torcs_ros/trajectory_ctrl")
#                            rospy.sleep(1) #give the system enough time to have killed of node
#                            RestartTORCSRace()
##                            os.system("roslaunch torcs_ros_trajectory_ctrl torcs_ros_trajectory_ctrl.xml") #relaunch client node with roslaunch command when in namespace (if launched with bringup .launch file)
##                            rospy.sleep(1)
#                            os.system("roslaunch torcs_ros_client torcs_ros_client_only.xml") #relaunch client node with roslaunch command when in namespace (if launched with bringup .launch file)
#                            self.n_counterConnection = 0
#                    else:
#                        self.n_counterConnection = 0
                if("Server did not respond in 1 second" in msg.msg):
                    dt = rospy.Time.now() - self.log_callback_time_pause 
                    self.log_callback_time_pause = rospy.Time.now()
                    if(dt.secs < 3):
                        self.n_counterPause += 1
                        if(self.n_counterPause >= self.n_counterComp):
                            print("\033[96mWatchdog is wanting to unpause TORCS \033[0m")
    #                        rospy.sleep(5) #give enough time to manually stop this process
#                            subprocess.call('xdotool search --name "torcs-bin" key p', shell=True) 
                            self.pub_demandPause.publish(Bool())
                            self.n_counterPause = 0
                    else:
                        self.n_counterPause = 0
#        
                        #handle case where game is connected but 

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
                    os.system("roslaunch torcs_ros_client torcs_ros_client_only.xml")
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
                os.system("roslaunch torcs_ros_client torcs_ros_client_only.xml") #relaunch client node with roslaunch command when in namespace (if launched with bringup .launch file)



if __name__ == "__main__":
    rospy.init_node("torcs_ros_client_watchdog")
    watchdog = Watchdog()
    while not rospy.is_shutdown(): 
#        watchdog.is_client_alive() #check whether client is dead
#        watchdog.has_client_connected() #check whether client is connected to torcs server
        rospy.sleep(1)
        
        
# Not connected to server yet!!
