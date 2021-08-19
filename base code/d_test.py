# ************************************
#   First Try at : Visual Odometry   #
# ************************************
# 					     Dhruv Patel #
# ************************************
#          University of Cincinnati  #
# ************************************


import numpy as np 
import cv2 as ai

import os

from d_visual_odometry import PinholeCamera, VisualOdometry


# Define the Path for the img folder 
img_folder_path = '/Volumes/Mr. DBKHD/Dataset/KITTI/pose_dataset/sequences/00/image_0'

# Calculating total number of files
def listdir_nohidden(path):
    count = 0
    for f in os.listdir(path):
        if not f.startswith('.'):
            count += 1
    return count

img_count = listdir_nohidden(img_folder_path)



traj = np.zeros((600,600,3), dtype=np.uint8)


# ****************************
# Pin - Hole Camera Model
# ****************************

cam = PinholeCamera(1241.0, 376.0, 718.8560, 718.8560, 607.1928, 185.2157)


# ************************
# Visual Odometry
# ************************

vo = VisualOdometry(cam, '/Volumes/Mr. DBKHD/Dataset/KITTI/true_dataset/poses/00.txt')



for img_id in range(img_count):
	img = ai.imread('/Volumes/Mr. DBKHD/Dataset/KITTI/pose_dataset/sequences/00/image_0/'+str(img_id).zfill(6)+'.png', 0)

	vo.pointer(img, img_id)

	cur_t = vo.cur_t

	if(img_id > 2):
		x, y, z = cur_t[0], cur_t[1], cur_t[2]
	else:
		x, y, z = 0.0, 0.0, 0.0

	draw_x, draw_y = int(x)+290, int(z)+90
	true_x, true_y = int(vo.true_x)+290, int(vo.true_z)+90

	ai.circle(traj, (draw_x,draw_y), 1, ( img_id * 255/4540, 255 - img_id * 255/4540,0), 1 )
	ai.circle(traj, (true_x,true_y), 1, (0,0,255), 2)
	ai.rectangle(traj, (10, 20), (600, 60), (0,0,0), -1)
	text = "Coordinates: x=%2fm y=%2fm z=%2fm"%(x,y,z)
	ai.putText(traj, text, (20,40), ai.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1, 8)

	ai.imshow('Road facing camera', img)
	ai.imshow('Trajectory', traj)
	ai.waitKey(1)

ai.imwrite('map.png', traj)
