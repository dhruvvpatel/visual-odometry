# ---------------------------------------------------
# - Making Base Code for Visual - Inertial Odometry -
# ---------------------------------------------------
#            dhruv patel | University of Cincinnati
# ---------------------------------------------------


import os
import time
import math

import numpy as np
import matplotlib.pyplot as plt


# data from IMU ??? : input
'''
based on the input we can choose a method
1. Euler Angles to find Rotational Matrix
2. Using quartanions to find Rotational Matrix

'''
# Euler angles
pitch = theta = 1
yaw = shi = 2
roll = phi = 3

g_b = 9.81 # place holder

# -------------------------------------------------------------
# Calculating gravity in the world frame from the body frame. |
# -------------------------------------------------------------

# Getting the rotational matrix

r11 = np.cos(phi) * np.cos(theta)
r12 = -np.cos(shi) * np.sin(theta) + np.sin(shi) * np.sin(phi) * np.cos(theta)
r13 = np.sin(shi) * np.sin(theta) + np.cos(shi) * np.sin(phi) * np.cos(theta)

r21 = np.cos(phi) * np.sin(theta)
r22 = np.cos(shi) * np.cos(theta) + np.sin(shi) * np.sin(phi) * np.sin(theta) 
r23 = -np.sin(shi) * np.cos(theta) + np.cos(shi) * np.sin(phi) * np.sin(theta)

r31 = -np.sin(phi) 
r32 = np.sin(phi) * np.cos(shi)
r33 = np.cos(phi) * np.cos(theta)

R_wb = np.array([ [r11, r12, r13],
									[r21, r22, r23],
									[r31, r32, r33] ])


print(R_wb)

# Gravity in the world frame can be obtained using information from the body frame.

g_w = R_wb * g_b



# 3-axis accelerometer gives g_b

'''
g_b = np.array([ [g_bx],
								 [g_by],
								 [g_bz] 	])

g_b = R_bw * g_w

where, g_w = np.array([ [0],
												[0],
												[g]		])

'''

g_b = np.array([ [ -g * np.sin(theta) ],
								 [ g * np.cos(shi) * np.sin(phi) ],
								 [ g * np.cos(shi) * np.cos(theta) ]	])


# Solving for pitch and roll angles

theta = np.arctan2( g_by, np.sqrt( (g_bx)^2 + (g_bz)^2 ) )

phi = np.arctan2( - g_bx, g_bz )
 


# Equation to calculate position and velocity

''' 
Basic Accel and Vel equations
1. Update Velocity first based on the acceleration
2. Update Position based on the updated velocity

'''

V_b_k1 = V_b_k + a_bk * dt

S_b_k1 = S_b_k + V_bk * dt



# Now he is saying, IMU gives : velocity(v), position(S), angular velocity(omega), orientation(R) ???





# -------------------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------------------



# -----------------------------
# 			Visual Odometry       |
# -----------------------------


'''
This part is working...

Just need to copy it here or in the main code

'''


# -------------------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------------------

# -----------------------------
# 			 Sensor Fusion        |
# -----------------------------


'''
Question ::

1. What is the state ?

x = [position, velocity, q, omega]
---> position will have 3 values     | (px, py, pz)
---> same with velocity              | (vx, vy, vz)
---> quartanion has 4 values         | 1 x real & 3 x imaginary
---> omega will have 3 values        | (wx, wy, wz)

x.shape = [13, 1]


2. What is control input (u) 


3. Deciding process noise and measurement noise 


''' 


# Defining Prediction Function

def Prediction(x_t, P_t, F_t, B_t, U_t, G_t, Q_t):
	x_t = F_t.dot(x_t) + B_t.dot(U_t) 
	P_t = (G_t.dot(P_t).dot(G_t.T)) + Q_t
	
	return x_t, P_t


# Defining Update Function
def Update(x_t, P_t, Z_t, R_t, H_t):
	S = np.linalg.inv( (H_t.dot(P_t).dot(H_t.T)) + R_t )
	K = P_t.dot(H_t.T).dot(S)
	
	x_t = x_t + K.dot( Z_t - H_t.dot(x_t) )
	P_t = P_t - K.dot(H_t).dot(P_t)
	
	return x_t, P_t


'''
TO BE DECIDED | Kalman Filter Parameters
'''

# Transition Matrix
F_t = np.array([  [1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]  ])


# Initial Covariance State
P_t = 0.005 * np.identity(3)

# Process Covariance
Q_t = 0.004 * np.identity(3)

# Measurement Covariance
R_t = np.array([    [0.24, 0],
                    [0, 0.24]  ])

# Measurement Matrix
H_t = np.array([    [1, 0, 0],
                    [0, 1, 0]   ])

# Initial State
x_t = np.array([ [sen_pos_x[0]], [sen_pos_y[0]], [sen_pos_theta[0]] ])



kal_x, kal_y, kal_theta = [], [], []

