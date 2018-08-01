
#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun May 28 12:36:10 2017

@author: Zorn, Ronecker
"""

import h5py
import numpy as np
#import matplotlib.plt as plt
import time

do_manual = False; #determines control signal from control crosshair pixel values, used for manual driving data
path = "/home/ben/torcs/catkin_ws/training_data/logged_data-2017-10-9_12:38-ben.h5" #file to be converted
h5file = h5py.File(path, 'r+') #open
img_string_len = h5file['img_image'].shape[0] #length of data 

try: #create and resize datasets to maximum lenght
    h5file.create_dataset('img_image_array', (img_string_len,480,640,3), maxshape=(None,480,640,3),compression="gzip", compression_opts=9)
    #only needed for manual driving:
    h5file['ctrl_Accel'].resize((img_string_len,))  
    h5file['ctrl_Brake'].resize((img_string_len,))  
    h5file['ctrl_Stear'].resize((img_string_len,))  

except:
    pass
    print("Dataset for rgb data exists already. Continuing with script anyways")


h5file.close()

#%% Border pixels of control crosshair
x_cent = 559
y_cent = 399
x_left = 521
x_right = 598
y_up = 360
y_down = 438


BGR = [] #480x640x3 array; would be easier with numpy
for i in range(480):
    BGR.append([])
    for ii in range(640):
        BGR[i].append([])
        for iii in range(3):
            BGR[i][ii].append(0)   
h5file = h5py.File(path, 'r+')
img_string = h5file['img_image'] #image string data
prev_j = -1 #helper variable
cor = []; 
#arrays for accel, brake and steering
acc_array = [] 
brk_array = []
str_array = []

for j in range(img_string_len):

    tic = time.time()
    print(str(j)+ " of " + str(img_string_len-1)) 
    cur_img = BGR #copy empty array; would be better with numpy
    img = np.fromstring(img_string[j], np.uint8) #make uint8 array from string

    counter = 0
    show = [[],[],[]] #helper array

    #%% Append next uint8 value to proper rgb channel
    for f in range(len(img)):
        if counter == 0:
            show[0].append(img[f])
            counter = 1;
        elif counter == 1:
            show[1].append(img[f])
            counter = 2;
        elif counter == 2:
            show[2].append(img[f])
            counter = 0;
    

    show2 = np.array(np.transpose(show)) #transpose of show    
    row = 0 #row counter
    col = 0 #collumn counter

    #%% 
    for f in range(len(show2)-1):
        cur_img[row][col][0] = show2[f][0]
        cur_img[row][col][1] = show2[f][1]
        cur_img[row][col][2] = show2[f][2]
#        print(show2[f][2])
        col += 1
        if col >= 640: #end of column
            col = 0 #reset
            row +=1 #next row

        

    cur_img = np.array(cur_img)
    cur_img = np.ndarray.astype(cur_img, 'uint8')
#    imgplot = plt.imshow(cur_img) # display current image
    
    
    #%% Use pixel data to determine control signals
    # only needed for manual driving
    steering = 0
    accel = 0
    brake = 0
    if do_manual is True:
        for a in range(3, 38):
            if cur_img[y_cent, x_cent-a, 0] >= 250 and cur_img[y_cent, x_cent-a, 1] < 250:#steering left; <250 ensures pure blue (as crosshair is 255,255,255; pure white))
                steering = float(a)/38
    #            print(a)
            if cur_img[y_cent, x_cent+a, 0] >= 250 and cur_img[y_cent, x_cent+a, 1] < 250:#steering right 
    #            print(a)
                steering = -float(a)/38
            if cur_img[y_cent-a, x_cent, 0] >= 250 and cur_img[y_cent-a, x_cent, 1] < 250:# accel
                accel = float(a)/38
            if cur_img[y_cent+a, x_cent, 0] >= 250 and cur_img[y_cent+a, x_cent, 1] < 250:# brake
                brake = float(a)/38

    acc_array.append(accel)
    brk_array.append(brake)
    str_array.append(steering)

    cur_img = np.true_divide(cur_img, 255) #limit to 0 to 1
    toc = time.time()
    cor.append(cur_img)

    #store every 250 samples
    print("Time elapsed: " + str(toc-tic))
    if (j+1)%250 == 0: 
        print("Start storing")
        acc_array = np.array(acc_array)
        brk_array = np.array(brk_array)
        str_array = np.array(str_array)
        h5file['img_image_array'][j-249:j+1] = cor
        h5file['ctrl_Accel'][j-249:j+1] = acc_array
        h5file['ctrl_Brake'][j-249:j+1] = brk_array
        h5file['ctrl_Stear'][j-249:j+1] = str_array
        
        tic = time.time()
        print("Time needed for storing " + str(tic-toc))
        acc_array = np.array(acc_array)
        brk_array = np.array(brk_array)
        str_array = np.array(str_array)
        del(cor) 
        del(acc_array)
        del(brk_array)
        del(str_array)
        cor = []; 
        acc_array = []
        brk_array = []
        str_array = []
        prev_j = j

    elif j == img_string_len-1:
        print(j)
        print("Start storing last chunck")
        h5file['img_image_array'][prev_j+1:j+1] = cor
        h5file['ctrl_Accel'][prev_j+1:j+1] = acc_array
        h5file['ctrl_Brake'][prev_j+1:j+1] = brk_array
        h5file['ctrl_Stear'][prev_j+1:j+1] = str_array
        tic = time.time()
        print("Time needed for storing " + str(tic-toc))
        del(cor) 
        del(acc_array)
        del(brk_array)
        del(str_array)
        cor = []; 
        acc_array = []
        brk_array = []
        str_array = []



h5file.close()


