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
from std_msgs_stamped.msg import BoolStamped
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
        self.pub_isRestarted = rospy.Publisher("/torcs_ros/notifications/isRestarted", BoolStamped, queue_size=1) #publisher for flag that torcs has been restarted
        
        self.sub_demandPause = rospy.Subscriber("/torcs_ros/notifications/demandPause", BoolStamped, self.callback_pause, queue_size=1) #subscription for pause demands
        self.sub_demandRestart = rospy.Subscriber("/torcs_ros/notifications/demandRestart", BoolStamped, self.callback_restart, queue_size=1) #subscription for restart demands
        
        self.FNULL = open(os.devnull, 'w') #redirects output to not be published to console

    #pause game on callback unless game is currently being restarted
    def callback_pause(self, msg):
        #game does not have to be paused while it is in the process of being restarted (mutex handling)
        if (not self.b_beingRestarted): #check for restart flag
            self.b_beingPaused = True #set pause mutex
            self.b_Pause = not self.b_Pause #invert pause state
            subprocess.call('xdotool search --name "torcs-bin" key p', shell=True) #pause game
            self.b_beingPaused = False #unset pause mutex
        
    def callback_restart(self, msg):
        
        self.b_beingRestarted = True #set restart mutex
        while(self.b_beingPaused): #wait for pause callback to finish if it is happening currently, works as callback_pause is performed in another thread
            pass
        if(self.b_Pause): #check whether game is currently paused
            subprocess.call('xdotool search --name "torcs-bin" key p', shell=True) #unpause game
            self.b_Pause = not self.b_Pause#set flag to game being unpaused
            rospy.sleep(0.5) #ensure enough time has passed for game to be unpaused
        msg_ctrl = TORCSCtrl() #message container
        msg_ctrl.meta = 1 #set restart flag
        #ensure last sent control command is no movement
        msg_ctrl.accel = 0 
        msg_ctrl.steering = 0
        msg_ctrl.brake = 1

        n_iter = 0 #flag for different case handling
        #publish meta restart command until it has been processed by client (has to be repeated in case any message is chocked)
        while(True):
            msg_ctrl.header.stamp = rospy.Time.now()
            self.pub_ctrl.publish(msg_ctrl) #demand restart from client node
            try: 
                #check whether the restart has been engaged by client; timeout ensures that if the client is not running the except clause is hight
                msg_restartNotification = rospy.wait_for_message("/torcs_ros/notifications/restart_process", Bool, timeout=5) 
            except:
                msg_restartNotification = Bool() #needed as no message was received
                #client seems to have some issues.
                #case 1: game is paused which leads to client to not send any data
                #solution: unpause game
                if (n_iter == 0):
                    subprocess.call('xdotool search --name "torcs-bin" key p', shell=True) #unpause game
                    self.b_Pause = False #set flag to game being unpaused
                    rospy.sleep(0.5) #ensure enough time has passed for game to be unpaused
                    n_iter += 1 #go to next case if this does not fix the issue
                    msg_restartNotification.data = False #continue loop, if unpause fixes the issue another message with data==True wiill be received
                #case 2: the client has restarted the game but the notification message was chocked
                #solution: manually break from true-loop by setting flag
                else:
                    #as the previous unpausing did not fix the issue and was therefore most likely faulty
                    #reset the pause state to what it should be
                    self.b_Pause = not self.b_Pause 
                    msg_restartNotification.data = True #needed to exit loop

            if(msg_restartNotification.data == True):
                break #break loop as the meta command has been received


        subprocess.call("rosnode kill /torcs_ros/torcs_ros_client_node", shell = True,  stdout=self.FNULL) #kills client node with terminal command
        
        subprocess.Popen("roslaunch torcs_ros_client torcs_ros_client_only.xml", shell=True, stdout=self.FNULL, stderr=subprocess.STDOUT) #relaunch client node with roslaunch command when in namespace (if launched with bringup .launch file)
        #notify that game has been restarted
        msg_restart = BoolStamped() 
        msg_restart.header.stamp = rospy.Time.now()
        self.pub_isRestarted.publish(msg_restart)
        
        #unpause game after restart if it happens to be paused
        #ping client node to see whether it is up and running yet
        while(True):
#            string = os.popen("rosnode ping -c 1 /torcs_ros/torcs_ros_client_node &> /dev/null").readlines() #ping client and return string to variable 
#            if(np.array(["reply" in line for line in string]).any()): #node is up if "reply" is in any of the returnee lines
            string = subprocess.check_output("rosnode ping -c 1 /torcs_ros/torcs_ros_client_node", shell=True, stderr=subprocess.STDOUT)
            if ("reply" in string):
                if (self.b_Pause == True): #check current pause state and unpause game if it is paused
                    subprocess.call('xdotool search --name "torcs-bin" key p', shell=True) #unpause game
                    self.b_Pause = False  
                else:
                    try: #handle case were self.b_Pause was set wrong
                        rospy.wait_for_message("/torcs_ros/sensors_state", TORCSSensors, timeout=5) #see whether sensor messages are published, meaning game is unpaused
                        break
                    except: #on timeout: unpause game
                        subprocess.call('xdotool search --name "torcs-bin" key p', shell=True) #unpause game
                        self.b_Pause = False  
                        break
            else:
                rospy.sleep(0.1) #limit rate
     
        self.b_beingRestarted = False #unset restart mutex


if __name__ == "__main__":
    if(len(os.popen("dpkg -l | grep xdotool").readlines()) == 0): #check whether xdotool is installed
        print("\033[93mIt seems like xdotool is not installed! Please install it with pip as torcs_ros_client_gamestate relies on it. Tested on Ubuntu 16.04 with xdotool 3.20150503.1 \033[97m")

    rospy.init_node("game_state_node")
    gamestate = GameState()
    rospy.spin()