#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Aug  1 11:39:31 2018

@author: bzorn
"""

import rospy
import numpy as np
import os
import subprocess

from std_msgs.msg import Bool
from torcs_msgs.msg import TORCSCtrl, TORCSSensors
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Log


#A class that manages whether torcs is paused or restarted when demands are published
class GameState():
    def __init__(self):
        self.b_Pause = False #variable indicating current pause state
        self.b_beingRestarted = False #variable used for mutex callback, restart has priority
        self.b_beingPaused = False #variable used for mutex callback, restart has priority
        self.pub_ctrl = rospy.Publisher("/torcs_ros/ctrl_cmd", TORCSCtrl, queue_size=1) #publisher for meta restart
        self.pub_isRestarted = rospy.Publisher("/torcs_ros/isRestarted", Bool, queue_size=1) #publisher for flag that torcs has been restarted
        
        self.sub_demandPause = rospy.Subscriber("/torcs_ros/demandPause", Bool, self.callback_pause, queue_size=5) #subscription for pause demands
        self.sub_demandRestart = rospy.Subscriber("/torcs_ros/demandRestart", Bool, self.callback_restart, queue_size=1) #subscription for restart demands
        
    #pause game on callback unless game is currently being restarted
    def callback_pause(self, msg):
        if (not self.b_beingRestarted): #check for restart flag
            self.b_beingPaused = True #set pause mutex
            self.b_Pause = not self.b_Pause #invert pause state
            subprocess.call('xdotool search --name "torcs-bin" key p', shell=True) #pause game
            self.b_beingPaused = False #unset pause mutex
        
    def callback_restart(self, msg):
        self.b_beingRestarted = True #set restart mutex
        while(self.b_beingPaused): #wait for pause callback to finish if it is happening currently
            pass
        if(self.b_Pause): #check whether game is currently paused
            subprocess.call('xdotool search --name "torcs-bin" key p', shell=True) #unpause game
            self.b_Pause = not self.b_Pause#set flag to game being unpaused
            rospy.sleep(0.5)
        msg_ctrl = TORCSCtrl() #message container
        msg_ctrl.meta = 1 #set restart flag
        #ensure last sent control command is no movement
        msg_ctrl.accel = 0 
        msg_ctrl.steering = 0
        msg_ctrl.brake = 1
#        self.pub_ctrl.publish(msg_ctrl) #demand restart from client node
#        rospy.sleep(10) #wait to be sure that client has restarted
        while(True): #this is needed as sometimes the client will seems to be chocking messages, which leads to the client being killed before torcs is restarted
            self.pub_ctrl.publish(msg_ctrl) #demand restart from client node
            msg_restartNotification = rospy.wait_for_message("/torcs_ros/restart_process", Bool)
            if(msg_restartNotification.data == True):
                break
            
#        while(True):
##            pass
#            msg_warn = rospy.wait_for_message("/rosout", Log)
#            if ("Client Restart" in msg_warn.msg):
#                break
        FNULL = open(os.devnull, 'w') #redirects output to not be published to console
        subprocess.call("rosnode kill /torcs_ros/torcs_ros_client_node", shell = True)#,  stdout=FNULL) #kills client node with terminal command
        
        
        subprocess.Popen("roslaunch torcs_ros_client torcs_ros_client_only.xml", shell=True)#, stdout=FNULL, stderr=subprocess.STDOUT) #relaunch client node with roslaunch command when in namespace (if launched with bringup .launch file)
        #notify that game has been restarted
        msg_restart = Bool() 
        self.pub_isRestarted.publish(msg_restart)
        self.b_beingRestarted = False #unset restart mutex

if __name__ == "__main__":
    if(len(os.popen("dpkg -l | grep xdotool").readlines()) == 0): #check whether xdotool is installed
        print("\033[93mIt seems like xdotool is not installed! Please install it with pip as torcs_ros_client_gamestate relies on it. Tested on Ubuntu 16.04 with xdotool 3.20150503.1 \033[97m")

    rospy.init_node("game_state_node")
    gamestate = GameState()
    rospy.spin()