import numpy as np 
import cv2 as ai
import os


# Flag for the frames
STAGE_FIRST_FRAME = 0
STAGE_SECOND_FRAME = 1
STAGE_DEFAULT_FRAME = 2

# Minimum Feature below which Re-detection would be triggered
kMinNumFeature = 1500

# Scaling threshold that will determine when to multiply scale with R, t
scaling_threshold = 0.1



# ************************
# Feature Tracking
# ************************

# Parameter for the OpticalFlow using Lucas-Kande method with pyramids
lk_params = dict(winSize = (21,21),
                 criteria = (ai.TERM_CRITERIA_EPS | ai.TERM_CRITERIA_COUNT, 30, 0.01))


def featureTracking(prev_img, now_img, prev_px):

    new_kp, st, err = ai.calcOpticalFlowPyrLK(prev_img, now_img, prev_px, None, **lk_params)

    st = st.reshape(st.shape[0])

    # Choosing the best feature as we move forward
    prev_kp = prev_px[st == 1]
    new_kp = new_kp[st == 1]

    return prev_kp, new_kp


# ************************
# Pinhole Camera Model
# ************************

class PinholeCamera:

    def __init__(self, width, height, fx, fy, cx, cy,
                 k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0, k3 = 0.0):

        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        
        self.distortion = abs(k1) > 0.0000001
        self.d = [k1, k2, p1, p2, k3]


# ************************
# Visual Odometry 
# ************************

class VisualOdometry:

    def __init__(self, cam, annotations):

        self.frame_stage = 0
        self.cam = cam                                  # Pinhole Camera Model that we created
        self.now_frame = None                           # New incoming frame
        self.prev_frame = None                          # Previous frame 
        self.cur_R = None                               # Rotation Matrix
        self.cur_t = None                               # Translation Vector
        self.prev_px = None                             # Features extracted using detector : from previous frame
        self.now_px = None                              # Features extracted using detector : from new incoming frame
        self.focal = cam.fx                             # Camera focal length
        self.cc = ( cam.cx, cam.cy )                    # Camera Center
        self.true_x, self.true_y, self.true_z = 0, 0, 0
        # Setting the detector :: From OpenCV library 
        self.detector = ai.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)

        with open(annotations) as f:
            self.annotations = f.readlines()

    # Specialized for KITTI odometry dataset
    def getAbsoluteScale(self, frame_id):

        ss = self.annotations[frame_id - 1].strip().split()
        x_prev = float(ss[3])
        y_prev = float(ss[7])
        z_prev = float(ss[11])

        ss = self.annotations[frame_id].strip().split()
        x = float(ss[3])
        y = float(ss[7])
        z = float(ss[11])

        self.true_x, self.true_y, self.true_z = x, y, z

        abs_scale = np.sqrt( (x - x_prev)**2 + (y - y_prev)**2 + (z - z_prev)**2 )

        return abs_scale

    
    # Process First Frame :: 
    # --------------------------------------------------------
        # What to do when the very first frame arrives ?
        # 1. Detect features using the specified detector
        # 2. convert the detected features into an np.array
        # 3. and set the flag for second frame to :: TRUE
    # --------------------------------------------------------

    def processFirstFrame(self):
        self.prev_px = self.detector.detect(self.now_frame)
        self.prev_px = np.array([x.pt for x in self.prev_px], dtype=np.float32)
        self.frame_stage = STAGE_SECOND_FRAME


    # Process Second Frame :: 
    # --------------------------------------------------------
        # What to do when the second frame arrives ?
        # 1. Track features from prev_img to now_img and 
        #    forward only the good features to next steps.
        # 2. Find EssentialMatrix from features that we got 
        #    from feature tracking
        # 3. recover Pose :: R, t :: from essential matrix
        # and set flag for default frame to :: TRUE
        # also save current good key-points to prev key-points
    # --------------------------------------------------------

    def processSecondFrame(self):
        self.prev_px, self.now_px = featureTracking(self.prev_frame, self.now_frame, self.prev_px)
        E, mask = ai.findEssentialMat(self.now_px, self.prev_px, focal=self.focal, pp=self.cc, method=ai.RANSAC, prob=0.999, threshold=1.0)
        _, self.cur_R, self.cur_t, mask = ai.recoverPose(E, self.now_px, self.prev_px, focal=self.focal, pp=self.cc)
        self.frame_stage = STAGE_DEFAULT_FRAME
        self.prev_px = self.now_px


    # Process Normal Frame :: 
    # --------------------------------------------------------
        # What to do when the instantenous frame arrives ?
        # 1. Track features from prev_img to now_img and 
        #    forward only the good features to next steps.
        # 2. Find EssentialMatrix from features that we got 
        #    from feature tracking
        # 3. recover Pose :: R, t :: from essential matrix
        # ---------- SAME AS PROCESS_SECOND_FRAME ------------
        # 4. Calculate Absolute Scale from frame_id
        # 5. If abs_scale is greater than some threshold, then
        #    multiply it to R, t and get scaled R, t
        # 6. If key-points go under a certain limit, then
        #    trigger Re-Detection and convert them in an np.array
        # also save current good key-points to prev key-points
    # --------------------------------------------------------   

    def processFrame(self, frame_id):
        self.prev_px, self.now_px = featureTracking(self.prev_frame, self.now_frame, self.prev_px)
        E, mask = ai.findEssentialMat(self.now_px, self.prev_px, focal=self.focal, pp=self.cc, method=ai.RANSAC, prob=0.999, threshold=1.0)
        _, R, t, mask = ai.recoverPose(E, self.now_px, self.prev_px, focal=self.focal, pp=self.cc)

        abs_scale = self.getAbsoluteScale(frame_id)

        if(abs_scale > scaling_threshold):
            self.cur_t = self.cur_t + abs_scale*self.cur_R.dot(t)
            self.cur_R = R.dot(self.cur_R)


        if(self.prev_px.shape[0] < kMinNumFeature):
            self.now_px = self.detector.detect(self.now_frame)
            self.now_px = np.array([x.pt for x in self.now_px], dtype=np.float32)
        
        self.prev_px = self.now_px

    # Update Step :: 
    # --------------------------------------------------------
        # Pointer fucntions :: points to what to do next
    # --------------------------------------------------------  

    def pointer(self, img, frame_id):

        assert(img.ndim==2 and img.shape[0]==self.cam.height and img.shape[1]==self.cam.width), "Frame: provided image has not the same size as the camera model or image is not grayscale"

        self.now_frame = img

        if(self.frame_stage == STAGE_DEFAULT_FRAME):
            self.processFrame(frame_id)
        
        elif(self.frame_stage == STAGE_SECOND_FRAME):
            self.processSecondFrame()
        
        elif(self.frame_stage == STAGE_FIRST_FRAME):
            self.processFirstFrame()
        
        self.prev_frame = self.now_frame
        
        
