#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 16 11:47:39 2017

@author: ben
"""
import os
## force gpu number
os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = "3" #slot number
import tensorflow as tf
import keras.backend.tensorflow_backend as ktf

import h5py 
import numpy as np
import keras
from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation, Flatten,  Conv2D, MaxPooling2D
from keras.optimizers import Adam
from keras import optimizers
from keras.callbacks import ModelCheckpoint
from sklearn.model_selection import train_test_split
import datetime as dt
from shutil import copyfile
from time import sleep

    
#%%limit GPU memory
from keras import backend as K
config = tf.ConfigProto()
config.gpu_options.allow_growth=True
sess = tf.Session(config=config)
K.set_session(sess)


#%% setup model
def model_setup():
    model = Sequential()
    l0= model.add(Conv2D(24, (5, 5), strides=(2,2), input_shape=(480, 640, 3), activation='elu')) #input_shape needed only for first layer
    l1 = model.add(Conv2D(36, (5, 5), strides=(2,2), activation='elu')) 
    l2 = model.add(Conv2D(48, (5, 5), strides=(2,2), activation='elu')) 
    l3 = model.add(Conv2D(64, (3, 3), activation='elu')) 
    l4 = model.add(Conv2D(64, (3, 3), activation='elu')) 
    l5 = model.add(Flatten())
    l6 = model.add(Dense(200, activation='elu'))
    ld6 = model.add(Dropout(0.5))
    l7 = model.add(Dense(100, activation='elu'))
    ld7 = model.add(Dropout(0.5))
    l8 = model.add(Dense(50, activation='elu'))
    ld8 = model.add(Dropout(0.5))
    l9 = model.add(Dense(10, activation='elu'))
    ld9 = model.add(Dropout(0.5))
    lout = model.add(Dense(1, activation='linear'))
    return model
    
    
#%% load data
class load_data:
    
    def __init__(self, files, out_string):
        #initialize data array
        self.data = {}
        self.data['image'] = []
        self.data['accel'] = []
        self.data['brake'] = []
        self.data['steering'] = []
        num_rounds = 3; #amount of rounds from every map
        tresh = 0.2 #treshhold value used for defining curvy samples after num_rounds
        
        for f in range(len(files)): #iterate over .h5 training data files
            h = h5py.File(files[f], 'r') #open current file
            print("Importing first " + str(num_rounds) + " rounds of " + str(files[f])) #display current map to be imported
            rtimes = h['sen_lapTimes'][:] #round times
            ctimes = h['sen_currentLapTime'][:] #current lap times
            round_count = 0; #counter variable for rounds
            counter = 0; #counter variable for imported data length
            counter_2 = 0; #counter variable for curvy data length
            case_b = False #bool variable for curvy data case
            for g in range(len(ctimes)): #iterate over all data in current file
                if round_count == num_rounds: #see whether maximum amount of rounds is reached
                    mask = range(counter) #creates ascending list; first round will always be taken
                    case_b = True #set bool to true
                if ctimes[g] >= 0: #ensure that lap time is positive 
                    counter += 1; 
                if  ctimes[g] == rtimes[round_count]: #if current lap time is next round time
                    round_count += 1; #round counter
                    print("Round " + str(round_count))
                if case_b == True: #for curvy data samples after num rounds is reached
                    if abs(h['ctrl_Stear'][counter]) >= tresh: #check whether current sample's stearing is above treshold
                        mask.append(counter) #append current sample to selection mask
                        counter_2 += 1; #counter for curcy data

            print(str(counter_2) +" of " + str(len(ctimes)) + "Data Samples for the current map have been identified as being over " + str(tresh)) #display amount of curvy data
        #%% Handle data input. Image will be imported as uint8
        ### Case Handling only needed for dimension adherence and proper concatenation; can be handled cleaner but should work
        if f == 0:
                self.data['image'].append(np.ndarray.astype(np.asarray(h['img_image_array'][mask]*255), 'uint8'))
                if out_string == 'accel':
                    self.data['accel'].append(np.asarray(h['ctrl_Accel'][mask]))
                elif out_string == 'brake':
                    self.data['brake'].append(np.asarray(h['ctrl_Brake'][mask]))
                else:
                    self.data['steering'].append((h['ctrl_Stear'][mask]))
        elif f == 1:
            if out_string == 'accel':
                self.data['accel'] = np.concatenate((self.data['accel'][0], np.asarray(h['ctrl_Accel'][mask])))
            elif out_string == 'brake':
                self.data['brake'] = np.concatenate((self.data['brake'][0], np.asarray(h['ctrl_Brake'][mask])))
            else:
                self.data['steering'] = np.concatenate((self.data['steering'][0], np.asarray(h['ctrl_Stear'][mask])))
            self.data['image'] = np.concatenate((self.data['image'][0], np.ndarray.astype(np.asarray(h['img_image_array'][mask]*255), 'uint8')), axis=0)
        else: 
            if out_string == 'accel':
                self.data['accel'] = np.concatenate((self.data['accel'], np.asarray(h['ctrl_Accel'][mask])))
            elif out_string == 'brake':
                self.data['brake'] = np.concatenate((self.data['brake'], np.asarray(h['ctrl_Brake'][mask])))
            else:
                self.data['steering'] = np.concatenate((self.data['steering'], np.asarray(h['ctrl_Stear'][mask])))
            self.data['image'] = np.concatenate((self.data['image'],np.ndarray.astype(np.asarray(h['img_image_array'][mask]*255), 'uint8')), axis=0)


            h.close() #close training data
            
        self.data['image'] = np.asarray(self.data['image']) 

        if f != 0: #expand dimension for input handling
            self.data['image'] = np.expand_dims(self.data['image'], axis=0) #if more than 1
        self.data['accel'] = np.asarray(self.data['accel'])
        self.data['brake'] = np.asarray(self.data['brake'])
        self.data['steering'] = np.asarray(self.data['steering'])
        if f == 0: #input handling if only one map is taken
            self.data['accel'] = self.data['accel'][0] 
            self.data['brake'] = self.data['brake'][0] 
            self.data['steering'] = self.data['steering'][0]
        print(self.data['image'].shape) #print ibput shape
        print(self.data[out_string].shape) #print output shape
        
        #shuffle data: no dimension given = shuffle first dimension
        #arrays to ensure that data is shuffled correclty
        a = [] 
        b = []
        for l in range(len(self.data[out_string])):
            a.append(l)
            b.append(-l)
   
    
        print("Starting to shuffle data")
        rng_state = np.random.get_state() #get current random state
        np.random.shuffle(self.data['accel'])
        np.random.set_state(rng_state) #reset to previous state
        np.random.shuffle(self.data['brake'])
        np.random.set_state(rng_state)
        np.random.shuffle(self.data['steering'])
        np.random.set_state(rng_state)
        np.random.shuffle(self.data['image'])
        np.random.set_state(rng_state)
        np.random.shuffle(a)
        np.random.set_state(rng_state)
        np.random.shuffle(b)
        c = [sum(x) for x in zip(a,b)] #plus and minus data are added elemntwise
        if max(c) == 0: #if all data is zero, every array has been shuffled identically
            print("Data has been accurately shuffled")
        else:
            print("Shuffle was not accurate")
        
        #use last 20% for validation and first 80% for test since it's shuffled

#%% data generator for training data

def train_generator(data, batch_size, split_rate, epoch_steps, out_string):
    last = 1; #variable used after iteration to mirror other 50 percent of data
    while(1): #True loop for generator
        if last == 0:
            counter = 1; #variable used for mirroring (1 := Mirror Data)
            last = 1;
        else:
            counter = 0;
            last = 0;
        
        for f in range(int(epoch_steps/2)): # divided by 2 as epoch steps includes mirrored data, this function called twice
            #xtrain and ytrain data
        
            strt_idx = f*batch_size; #start index for generator batch
            if f*batch_size < epoch_steps*batch_size - batch_size: #check whether next batch extends beyond borders
                end_idx = f*batch_size+batch_size #end index for next generator batch
            else: 
                end_idx = int(np.ceil((1-split_rate)*len(data.data[out_string]))) #limit index to end of array

            X_train = np.true_divide(data.data['image'][0][strt_idx:end_idx], 255) #limit image between 0 and 1

            if out_string != 'steering': #check which output should be inspected
                y = data.data[out_string][strt_idx:end_idx] #same signal for accel and brake; no mirroring
            else :
                if counter == 1: #mirror data
                    y = (-1*data.data[out_string][strt_idx:end_idx]+1)/2 #mirror (*-1) and limit between [0,1] instead of [-1,1]
                else:
                    y = (data.data[out_string][strt_idx:end_idx]+1)/2 #limit between [0,1]

            if counter == 1: #mirror image
                for g in range(len(X_train)):
                    X_train[g] = np.fliplr(X_train[g]) #mirror image (flip collumns)

            y_train = np.asarray(y) #y data as numpy array
            y_train = y_train.reshape(len(y_train),1) #reshape for neural network 

            counter = counter + 1 #update mirror variable
            if counter == 2: #reset mirror variable
                counter = 0
            yield X_train, y_train #yield generator batch
    return    
 
#%% data generator for test data
# basically identical to test generator
# only indices are different

def test_generator(data, batch_size2, split_rate, val_steps, out_string):
    last = 1;
    while(1):
        if last == 0:
            counter = 1;
            last = 1;
        else:
            counter = 0;
            last = 0;  

        for ff in range(int(val_steps/2)):

            end_idx2 = len(data.data[out_string])-(ff*batch_size2) #start end index from end of array
            if ff*batch_size2 < val_steps*batch_size2-batch_size2: #see whether batch extends beyond array limit
                strt_idx2 = end_idx2-batch_size2 #start index dependent on end index and batch size
            else:
                strt_idx2 = int(np.ceil((1-split_rate)*len(data.data[out_string])))
            X_test = np.true_divide(data.data['image'][0][strt_idx2:end_idx2], 255)
            if out_string != 'steering':
                y = data.data[out_string][strt_idx2:end_idx2]
            else:
                if counter == 1:
                    y = (-1*data.data[out_string][strt_idx2:end_idx2]+1)/2 #normalization for steering 
                else:
                    y = (data.data[out_string][strt_idx2:end_idx2]+1)/2

            if counter == 1: #mirror image and steering signal
                for g in range(len(X_test)):
                    X_test[g] = np.fliplr(X_test[g]) #mirror image
                        
            y_test = np.asarray(y)
            y_test = y_test.reshape(len(y_test),1)
            counter = counter + 1
            if counter == 2:
                counter = 0
            yield (X_test, y_test)
    return    
    
#%% Callback for end of epoch
class printepoch_Callback(keras.callbacks.Callback):
    # Initialize  logging 
    def on_train_begin(self, logs={}): 
        self.folder = dt.datetime.now();
        self.folder = '/raid/student_data/PP_TORCS_DL_1/model_checkpoints/x/' + str(self.folder.month) + "-" + str(self.folder.day) + "-" +str(self.folder.hour) + "/"
        if not os.path.exists(self.folder):
            os.makedirs(self.folder)
        self.log = self.folder + "log_file.h5"

    # Call after every epoch
    def on_epoch_end(self, epoch, logs={}):
        save_every_x_epochs = 5; #save every x epochen
        self.epoch = epoch;

        if epoch % save_every_x_epochs == 0: #save if x epoch
            self.path = self.folder + "weights_epoch_" + str(epoch) + ".h5" #define new name
            self.model.save(self.path) #save model

        return
    
    # Call at train end
    def on_train_end(self, logs={}): 
        self.model.save(self.folder + "weights_on_end.h5") #save model
        


batch_size = 5 
split_rate = 0.2 #train and test split
out_string = 'steering' # string used to determine which output to train on; either steering, accel or brake


model = model_setup() #Initialize neural net
path2 = '/raid/student_data/PP_TORCS_DL_1/training_data/' #Folder for all input data (please only have .h5 files in this folder)
h5files = os.listdir(path2) #get all files in path
for f in range(len(h5files)):
    h5files[f] = path2 + h5files[f] #append to path
dataC = load_data(h5files, out_string); #load data
sample_size = len(dataC.data[out_string]); #entire sample size of all data
if (sample_size*(1-split_rate)) % batch_size == 1: #if last batch is only one sample
    e_steps = np.floor(float(sample_size*(1-split_rate))/batch_size) #calculate steps per epoch dependent on split rate and batch size(round down)
else:
    e_steps = np.ceil(float(sample_size*(1-split_rate))/batch_size) #calculate steps per epoch on split rate and batch size (round up)

epoch_steps = e_steps*2; #double epoch steps, as data will be mirrored
val_steps = int(np.ceil(split_rate*(sample_size /batch_size))*2) #validation steps is dependent on epoch steps, batch size and split rate

optimizer_ent = keras.optimizers.Adadelta(lr=0.5, rho=0.95, epsilon=1e-08, decay=0.0) #default Adadelta optimizer

model.compile(loss='mean_squared_error', optimizer=optimizer_ent) #mse taken from paper
ep = printepoch_Callback() #initialize callback class


#%% Training
history = model.fit_generator(train_generator(dataC, batch_size, split_rate, epoch_steps, out_string), steps_per_epoch=epoch_steps, epochs = 15, validation_data=test_generator(dataC, batch_size, split_rate, val_steps, out_string), validation_steps=val_steps, verbose=1, callbacks=[ep], max_q_size = 200) 


#%% Write Loss history log file
h = h5py.File(ep.log, 'w')
h.create_dataset('Loss', (len(history.history['loss']),), maxshape=(None,), compression="gzip", compression_opts=9)
h.create_dataset('Val_Loss', (len(history.history['val_loss']),), maxshape=(None,), compression="gzip", compression_opts=9)
#print(history.history)
#print(history.history['loss'])
h['Loss'][:] = history.history['loss']
h['Val_Loss'] [:]= history.history['val_loss']
h.close()

#
#model_setup()