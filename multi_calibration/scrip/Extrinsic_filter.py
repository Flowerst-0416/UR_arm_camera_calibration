#!/usr/bin/env python3

import numpy as np
import cv2
import os
from os import listdir
from pathlib import Path
import glob
import yaml
import os

#Initialize folder directories
Home_path = os.path.expanduser('~') 
image_path = glob.glob(Home_path +"/Calibration_ws/src/UR_arm_camera_calibration/multi_calibration/laser/*.png")

with open(os.path.expanduser(Home_path +"/Calibration_ws/src/UR_arm_camera_calibration/multi_calibration/cfg/laser_calib.yaml"),'r') as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        for i in range(2):
            _ = file.readline()
        para = yaml.load(file, Loader=yaml.FullLoader)

if not os.path.exists(Home_path +"/Calibration_ws/src/UR_arm_camera_calibration/multi_calibration/laser/Target_Image"):
    os.makedirs(Home_path +"/Calibration_ws/src/UR_arm_camera_calibration/multi_calibration/laser/Target_Image")
    
if not os.path.exists(Home_path +"/Calibration_ws/src/UR_arm_camera_calibration/multi_calibration/laser/Unidentified"):
    os.makedirs(Home_path +"/Calibration_ws/src/UR_arm_camera_calibration/multi_calibration/laser/Unidentified")

if not os.path.exists(Home_path +"/Calibration_ws/src/UR_arm_camera_calibration/multi_calibration/laser/Noise_Image"):
    os.makedirs(Home_path +"/Calibration_ws/src/UR_arm_camera_calibration/multi_calibration/laser/Noise_Image")



#Define amount of each type
noiseCount = 0
targetCount = 0
count = 0
unidtfyCount = 0


#Set up red laser HSV range
hue_max = para['hue_max']
hue_min = para['hue_min']
sat_max = 255
sat_min = para['sat_min']
val_max = 255
val_min = para['val_min']
target_rows = para['target_rows']
target_cols = para['target_cols']

lower_red = np.array([hue_min,sat_min,val_min])
upper_red = np.array([hue_max,sat_max,val_max])

for image in image_path:
    print('Flitering... Finished:'+str( round((count/len(image_path))*100,2)) +'%')#Let the user know the image processing progress
    
    imageState = 0#Initialize image state, state = 0 means target image, otherwise noise image
    
    input_image = cv2.imread(image)
    retval, corners = cv2.findChessboardCorners(input_image, (target_rows,target_cols), flags=cv2.CALIB_CB_FAST_CHECK);
    
    input_image_HSV = cv2.cvtColor(input_image,cv2.COLOR_BGR2HSV)
    

    mask1 = cv2.inRange(input_image_HSV, lower_red, upper_red)
    
    red_pixels = np.argwhere(mask1)
    #red_pixels = np.vstack((np.argwhere(mask1),np.argwhere(mask2)))
    
   
    if retval == True:
        #cv2.drawChessboardCorners(input_image, (7, 10), corners, retval)
        for px, py in red_pixels:
            find = False
            for cy, cx in corners.reshape(70,2):
                diffx = abs(px-cx)
                diffy = abs(py-cy)
                if diffx <= 25 and diffy <= 25:
                    find = True #Make sure quit both for-loop with crossing is find
                    imageState += 1
                    cv2.circle(input_image, (py, px), 5, (0, 255, 255), 1)
                    break
            if find:
                break #Make sure quit both for-loop with crossing is find

                    
        if imageState == 0:
            
            os.chdir(Home_path +"/Calibration_ws/src/UR_arm_camera_calibration/multi_calibration/laser/Target_Image")
            cv2.imwrite("target-" + str(targetCount) + ".png",input_image)
            targetCount += 1
        else:
            os.chdir(Home_path +"/Calibration_ws/src/UR_arm_camera_calibration/multi_calibration/laser/Noise_Image")
            cv2.imwrite("noise-" + str(noiseCount) + ".png",input_image)
            noiseCount += 1
        
    elif retval == False:
        os.chdir(Home_path +"/Calibration_ws/src/UR_arm_camera_calibration/multi_calibration/laser/Unidentified")
        cv2.imwrite("unidentified-" + str(unidtfyCount) + ".png",input_image)
        unidtfyCount += 1
    
    count += 1

print('ready to start calibration')
