Visual-Odometry
----------------

## Odometry

Normally, a wheel odometer (or simply odometer ) measures the number of rotations that the wheel is undergoing, and multiplies that by the circumference to get an estimate of the distance travelled by the car. Odometry is used as a more general term in Robotics, and often refers to estimating not only the distance traveled, but the entire trajectory of a moving robot. So, for every time instance t, there is a vector [xt yt zt αt βt γt]T which describes the complete pose of the robot at that instance.
Note : αt, βt, γt here are the Euler Angles, while xt, yt, zt are Cartesian Coordinates of the robot.

## What is Visual Odometry ?

In Visual Odometry, we have a camera (or an array of cameras) rigidly attached to a moving object (such as a car or a robot), and our job is to construct a 6-DOF trajectory using the video stream coming from this camera(s). When only one camera is used, it’s called Monocular Visual Odometry and when two (or more) cameras, it’s referred to as Stereo Visual Odometry.

### Input:
We have a stream of images (i.e. video - grayscale/color) coming from a camera. We have prior knowledge of all the intrinsic parameters, obtained via calibration, which can be done in OpenCV.

### Output:
For every pair of images, we need to find the Rotation Matrix R and the Translation Vector t, which describes the motion of the vehicle between the two frames. The vector t can only be computed up to a scale factor in our monocular scheme.

## Working code:

The working code uses images from the KITTI dataset and ground truth for finding the absolute scale. 


## What are the next steps ??

- Determine what approach to be taken to get pose from IMU data.
	1)	Using Quartanions transformation
	2)	Using Euler Angles
	3)	.... and much more

- Infuse the Pose from IMU and VO using an EKF algorithm


## Questions

1)	What is the output of the IMU that we will get ??
2)	How to get the absolute scale for VIO ??
3)	Kalman Filter Equations 
4)	Optimization


