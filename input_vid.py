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


# ----------------------------------
# Reading : Images in a folder 
# ----------------------------------

# Define the Path for the img folder 
img_folder_path = '/Volumes/Mr. DBKHD/Dataset/KITTI/pose_dataset/sequences/10/image_0'

# Calculating total number of files
def listdir_nohidden(path):
    count = 0
    for f in os.listdir(path):
        if not f.startswith('.'):
            count += 1
    return count

img_count = listdir_nohidden(img_folder_path)

# --------------------------------------------------------------------
# --------------------------------------------------------------------

# ------------------------------
# Reading : A video file
# ------------------------------

# Opens the Video file
cap = ai.VideoCapture('/Users/phantom/fun/VO/code/1/seq_10.avi')

# Saving frames extracted from video in a folder
i = 0

# --------------------------------------------------------------------
# --------------------------------------------------------------------


traj_width = 800
traj_height = 400

traj = np.zeros((traj_width,traj_width,3), dtype=np.uint8)


# ****************************
# Pin - Hole Camera Model
# ****************************

cam = PinholeCamera(1241.0, 376.0, 718.8560, 718.8560, 607.1928, 185.2157)


# ************************
# Visual Odometry
# ************************

vo = VisualOdometry(cam, '/Volumes/Mr. DBKHD/Dataset/KITTI/true_dataset/poses/00.txt')


i = -1
while(cap.isOpened()):
	
	ret, frame = cap.read()
	frame = ai.cvtColor(frame, ai.COLOR_BGR2GRAY)

	if ret == True:
		i = i + 1
		img_id = i
	

	vo.pointer(frame, img_id)

	cur_t = vo.cur_t

	if(img_id > 2):
		x, y, z = cur_t[0], cur_t[1], cur_t[2]
	else:
		x, y, z = 0.0, 0.0, 0.0

	draw_x, draw_y = int(x)+400, int(z)+200
	true_x, true_y = int(vo.true_x)+400, int(vo.true_z)+200

	ai.circle(traj, (draw_x,draw_y), 1, ( img_id * 255/(img_count-1), 255 - img_id * 255/(img_count-1),0 ), 1 )
	ai.circle(traj, (true_x,true_y), 1, (0,0,255), 2)

	ai.rectangle(traj, (10, 20), (600, 60), (0,0,0), -1)
	
	text = "Coordinates: x=%2fm y=%2fm z=%2fm"%(x,y,z)
	ai.putText(traj, text, (20,40), ai.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1, 8)

	ai.imshow('Road facing camera', frame)
	ai.imshow('Trajectory', traj)
	ai.waitKey(1)

	if ret == False:
		break

cap.release()
ai.destroyAllWindows()

ai.imwrite('map.png', traj)

