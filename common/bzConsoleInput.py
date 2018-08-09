#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Aug  9 08:16:13 2018

@author: bzorn
"""

from enum import Enum
from bzReadYAML import readTrajectoryParams, saveYAML, readNengoHyperparams, readConfig
import os



class eUserIn(Enum):
    true = 1
    false = 0
    skip = -1
    default = -2
    invalid = -3

class cUserIn:
    def __init__(self, directory):
        self.directory = directory
        self.GetUserInput();
        
    def CastToEnum(self,i_chIn):
        if (i_chIn == 'y' or i_chIn == 'Y'):
            return eUserIn.true
        elif (i_chIn == 'n' or i_chIn == 'N'):
            return eUserIn.false
        elif (i_chIn == 's' or i_chIn == 'S'):
            return eUserIn.skip
        else:
            return eUserIn.invalid
        
    def CheckIntValid(self,i_nIn):
        if (i_nIn.value > 1 or i_nIn.value < -2):
            return False
        else:
            return True
    
    def GetInput(self,message, input_type, bSkipAll, *argv):
        
        for arg in argv:
            if arg in message:
                loc = message.find(arg)
                message = message[:loc] + "\033[96m" + message[loc:loc+len(arg)] + "\033[0m" + message[loc+len(arg):]
        if(not bSkipAll):
            bInputValid = False
            while(bInputValid == False):
                chUserIn = input_type(message + " (y)es/(n)o/(s)kip rest \033[0m\n")
                eUser = self.CastToEnum(chUserIn)
                bInputValid = self.CheckIntValid(eUser)
            if (eUser == eUserIn.skip):
                bSkipAll = True
            return [eUser, bSkipAll]
        return [eUserIn.default, bSkipAll]
            
    def GetInterpretedInput(self,defaultValue, message, *argv):
        for arg in argv:
            if arg in message:
                loc = message.find(arg)
                message = message[:loc] + "\033[96m" + message[loc:loc+len(arg)] + "\033[0m" + message[loc+len(arg):]
        nUserIn = input(message)
        if (nUserIn == False):
            return defaultValue
        else:
            return nUserIn
    
    def AskNengoParam(self,bSkipAll):
        [eNengoSave, bSkipAll] = self.GetInput("Do you want to save the trained nengo network:", raw_input, bSkipAll, "save", "nengo")
        [eNengoLoad, bSkipAll] = self.GetInput("Do you want to load from available nengo networks:", raw_input, bSkipAll, "load", "trained nengo")
        if (eNengoLoad == eUserIn.true):
            nengoPath = self.GetUserPath()
        else:
            nengoPath = ''
        [eNengoTrain, bSkipAll] = self.GetInput("Do you want to change any of the learning parameters:", raw_input, bSkipAll, "change", "learning parameters")
        if (eNengoTrain == eUserIn.true):
            self.ChangeLearningYaml()
        return [bSkipAll, eNengoSave, eNengoLoad, eNengoTrain, nengoPath]
    
    def AskDatalogger(self,bSkipAll):
        [eDatalogger, bSkipAll] = self.GetInput("Do you want to save the data:", raw_input, bSkipAll, "save", "data")
        return [bSkipAll, eDatalogger]
    
    def AskTrajectoryParam(self,bSkipAll):
        [eTrajectory, bSkipAll] = self.GetInput("Do you want to change trajectory parameters:", raw_input, bSkipAll, "change trajectory")
        if (eTrajectory == eUserIn.true):
            self.ChangeTrajectoryYaml()
        return [bSkipAll, eTrajectory]
    
    def ChangeTrajectoryYaml(self):
        [f_longitudinalDist, f_lateralDist, n_amount] = readTrajectoryParams(self.directory)
        f_longitudinalDist = self.GetInterpretedInput(f_longitudinalDist, "Current longitudinal distance is set to: " + str(f_longitudinalDist) +
                                 "\nWhat int would you like to set it to (False for unchanged):    ", "longitudinal distance", str(f_longitudinalDist))
        f_lateralDist = self.GetInterpretedInput(f_lateralDist, "Current lateral distance is set to: " + str(f_lateralDist) +
                                 "\nWhat int would you like to set it to (False for unchanged):    ", "lateral distance", str(f_lateralDist))
        n_amount = self.GetInterpretedInput(n_amount, "Current amount per category is set to: " + str(n_amount) +
                                 "\nWhat int would you like to set it to (False for unchanged):    ", str(n_amount), "amount per category")
        data = dict( f_longitudinalDist = f_longitudinalDist, f_lateralDist = f_lateralDist, n_amount = n_amount, b_visualizeAll = False)
        yamlPath = self.directory + "/torcs_ros_trajectory_gen/include/torcs_ros_trajectory_gen/trajectory_params.yaml"
        
        saveYAML(data, yamlPath)
        
    
    def ChangeLearningYaml(self):
        [f_epsilon_init, f_decay_factor, f_learning_rate, a_scanTrack] = readNengoHyperparams(self.directory)
        f_epsilon_init = self.GetInterpretedInput(f_epsilon_init, "Current epsilon init is set to: " + str(f_epsilon_init) +
                                 "\nWhat float would you like to set it to (False for unchanged):    ", "epsilon init ", str(f_epsilon_init))
        f_decay_factor = self.GetInterpretedInput(f_decay_factor, "Current epsilon decay is set to: " + str(f_decay_factor) +
                                 "\nWhat int would you like to set it to (False for unchanged):    ", "epsilon decay", str(f_decay_factor))
        f_learning_rate = self.GetInterpretedInput(f_learning_rate, "Current learning rate is set to: " + str(f_learning_rate) +
                                 "\nWhat float would you like to set it to (False for unchanged):    ", str(f_learning_rate), "learning rate")
        a_scanTrack = self.GetInterpretedInput(a_scanTrack, "Current rangefinder sensors are set to: " + str(a_scanTrack) +
                                 "\nWhat list would you like to set it to (False for unchanged); Please input as list [0, 1, 2]:   ", str(a_scanTrack), "rangefinder sensors")
        data = dict(f_epsilon_init = f_epsilon_init, f_decay_factor = f_decay_factor, f_learning_rate=f_learning_rate, a_scanTrack = a_scanTrack)
        yamlPath = self.directory + "/torcs_ros_trajectory_selection/include/torcs_ros_trajectory_selection/simulation_params.yaml"
        saveYAML(data, yamlPath)

    def GetUserInput(self):
        bSkipAll = False
        [bSkipAll, eDatalogger] = self.AskDatalogger(bSkipAll)
        [bSkipAll, eNengoSave, eNengoLoad, eNengoTrain, pathNengoParam] = self.AskNengoParam(bSkipAll)
        [bSkipAll, eTrajectory] = self.AskTrajectoryParam(bSkipAll)
        self.directory = self.directory[:-14] + "nengo_parameters"
        path = self.directory + "/config.yaml"
        if (not os.path.isfile(path)):
            eDatalogger = self.CompareWithDefault(eDatalogger.value, eUserIn.false.value)
            eNengoSave = self.CompareWithDefault(eNengoSave.value, eUserIn.true.value)
            eNengoLoad = self.CompareWithDefault(eNengoLoad.value, eUserIn.false.value)
        else:
            [eDataLogger_last, eNengoSave_last, eNengoLoad_last, pathNengo_last] = readConfig(path)
            eDatalogger = self.CompareWithDefault(eDatalogger.value, eDataLogger_last)
            eNengoSave = self.CompareWithDefault(eNengoSave.value, eNengoSave_last)
            eNengoLoad = self.CompareWithDefault(eNengoLoad.value, eNengoLoad_last)
            if(pathNengoParam == ''):
                pathNengoParam = pathNengo_last
        data = dict(data_log = eDatalogger, nengo_save = eNengoSave, nengo_load = eNengoLoad, nengo_weigths_path = pathNengoParam)
        saveYAML(data, path)

            
    def CompareWithDefault(self, value, default):
            if (value != eUserIn.skip.value and value != eUserIn.default.value):
                return value
            else:
                return default
        
    def GetUserPath(self):
        path = self.directory[:-14] + "nengo_parameters"
        folders = os.listdir(path)
        folders = [folder for folder in folders if os.path.isdir(path+"/"+folder)]
        folders.sort()
        print("Select a \033[96mfolder\033[0m with one of the following indices:")
        print("\033[96m-1\033[0m: newest weights available")
        for (folder, nCounter) in zip(folders, range(len(folders))):
            print("\033[96m%i\033[0m: "  % nCounter+ folder)
        folderSel = input()
        if (folderSel == -1):
            files =  os.listdir(path + "/" + folders[-1])
            files = [fileT for fileT in files if "meta" in fileT]
            files = [fileT[:-5] for fileT in files] 
            files.sort(key=lambda item: (len(item), item))
            print("Loading: " + path+"/"+files[-1])
            return (path+"/"+ folders[folderSel] + "/" + files[-1])
        print("Select a \033[96mfile\033[0m with one of the following indices:")
        files =  os.listdir(path + "/" + folders[folderSel])
        files = [fileT for fileT in files if "meta" in fileT]
        files = [fileT[:-5] for fileT in files]
        files.sort(key=lambda item: (len(item), item))
        for (fileT, nCounter) in zip(files, range(len(files))):
            print("\033[96m%i\033[0m: "  % nCounter + fileT)
        fileSel = input()
        return (path+"/"+folders[folderSel] + "/" + files[fileSel])

            
#[eNengoSave, bSkipAll] = GetInput("Do you want to save the nengo paramaters:", False, "save", "nengo")
