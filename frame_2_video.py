# ************************************
#      Convert Frames to Video       #
# ************************************
# 					     Dhruv Patel #
# ************************************
#          University of Cincinnati  #
# ************************************

import numpy as np
import cv2 as ai

import glob, os
 
 # Define the Path for the img folder 
img_folder_path = '/Volumes/Mr. DBKHD/Dataset/KITTI/pose_dataset/sequences/10/image_0/'

# Calculating total number of files
def listdir_nohidden(path):
    count = 0
    for f in os.listdir(path):
        if not f.startswith('.'):
            count += 1
    return count

img_count = listdir_nohidden(img_folder_path)

img_array = []
for img_id in range(img_count):
    img = ai.imread(img_folder_path+str(img_id).zfill(6)+'.png')
    height, width, layers = img.shape
    size = (width,height)
    img_array.append(img)
 
 
out = ai.VideoWriter('seq_10.avi',ai.VideoWriter_fourcc(*'DIVX'), 15, size)
 
for i in range(len(img_array)):
    out.write(img_array[i])

out.release()
