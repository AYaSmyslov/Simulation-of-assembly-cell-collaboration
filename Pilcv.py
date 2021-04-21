"""
What you need for start program:
    1. Launch Coppeliasim Edu
    2. Place on the stage:
        2.1 kinect;
        2.2 LBR_iiwa_7_R800.
    3. Install the required libraries:
        3.1 pip install opencv.
"""
class Motion_sim():
    def __init__(self):
        # Local import of modules
        import sim as vr
        self.sim = vr
        import numpy as np
        self.np = np

        import cv2 as cv2
        self.cv2 = cv2
        from PIL import Image as I
        self.I = I
        import array
        self.array = array
        import math as math
        self.math = math
        import time as time
        self.time = time
        import msgpack as msgpack
        self.msgpack = msgpack

        # Data for coppeliasim
        self.client_id = None
        self.step_done = True
        
        self.Vision_Sensor_Display_On = True # Checking whether the sensor is turned on

        # Handles
        self.v_handle_RGB = None # RGB sensor handle
        self.v_handle_depth = None # Depth sensor handle
        self.handles_KUKA = [-1, -1, -1, -1, -1, -1, -1] # Array of joint LBR_iiwa_7_R800 handles
        self.handle_grip = None # Grip handle
        self.handle_attach_point = None # Attach point handle on grip

        # Data
        self.img = [] # BGR(!!! not RGB) data (numpy array uint8)[height, width, id_color]
        self.depth = [] # Depth data (numpy array float64 (can be float32))[width, height]
        self.resol_img = [0, 0] # Resolution image [width, height]
        self.len_depth = 0 # Length depth map

        self.executedMovId='notReady'
        self.targetArm='LBR_iiwa_7_R800'
        self.stringSignalName=self.targetArm+'_executedMovId'

    # Start simulation
    def sim_Start(self):
        print ('Simulation started')
        self.sim.simxFinish(-1) # Just in case, close all opened connections
        self.client_id=self.sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
        if self.client_id!=-1:
            print ('Connected to remote API server')
        else:
            print ('Failed connecting to remote API server')
            print ('Program ended')
            exit(0)
        self.sim.simxSynchronous(self.client_id, True)
        self.sim.simxGetStringSignal(self.client_id, self.stringSignalName, self.sim.simx_opmode_streaming) #!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.sim.simxStartSimulation(self.client_id, self.sim.simx_opmode_blocking)


        # Get rgb and depth sensor handles
        sim_ok, self.v_handle_RGB = self.sim.simxGetObjectHandle(self.client_id, 'kinect_rgb', self.sim.simx_opmode_blocking) # Get RGB sensor handle
        self.check_sim_ok('kinect_rgb', sim_ok)
        sim_ok, self.v_handle_depth = self.sim.simxGetObjectHandle(self.client_id, 'kinect_depth', self.sim.simx_opmode_blocking) # Get depth sensor handle
        self.check_sim_ok('kinect_depth', sim_ok)
        # Init opmode
        sim_ok, self.resol_img, self.img = self.sim.simxGetVisionSensorImage(self.client_id, self.v_handle_RGB, 0 ,self.sim.simx_opmode_streaming)
        sim_ok, self.len_depth, self.depth = self.sim.simxGetVisionSensorDepthBuffer(self.client_id, self.v_handle_depth,self.sim.simx_opmode_streaming)

        # Get KUKA handle joints
        for i in range(1, 8):
            sim_ok, self.handles_KUKA[i-1] = self.sim.simxGetObjectHandle(self.client_id, 'LBR_iiwa_7_R800_joint'+str(i), self.sim.simx_opmode_blocking) # Get joint handle
            self.check_sim_ok('LBR_iiwa_7_R800_joint'+str(i), sim_ok)
            # Init opmode
            sim_ok, hpos = self.sim.simxGetJointPosition(self.client_id, self.handles_KUKA[i-1], self.sim.simx_opmode_streaming)
            self.sim.simxSetJointTargetPosition(self.client_id, self.handles_KUKA[i-1], 0, self.sim.simx_opmode_streaming)

        # Get grip open-close joint handle
        sim_ok, self.handle_grip = self.sim.simxGetObjectHandle(self.client_id,'RG2_openCloseJoint', self.sim.simx_opmode_blocking)
        self.check_sim_ok('RG2_openCloseJoint', sim_ok)
        self.sim.simxSetJointTargetVelocity(self.client_id, self.handle_grip, 0, self.sim.simx_opmode_streaming)
        self.sim.simxSetJointForce(self.client_id, self.handle_grip, 100, self.sim.simx_opmode_streaming)
        sim_ok, new_pos = self.sim.simxGetJointPosition(self.client_id, self.handle_grip, self.sim.simx_opmode_blocking)
        sim_ok, self.handle_attach_point = self.sim.simxGetObjectHandle(self.client_id,'RG2_attachPoint', self.sim.simx_opmode_blocking)
        
        

    # Output the return code to the console with message
    def check_sim_ok(self, msg, sim_code):
        (print(msg, 'ok') if sim_code == self.sim.simx_return_ok else print(msg, 'fail'))

    # Setting the joint position
    def set_pos(self, joint, trarget_angle):
        sim_ok, hpos = self.sim.simxGetJointPosition(self.client_id, joint, self.sim.simx_opmode_buffer)
        hpos = hpos * 180 / self.math.pi
        while abs(trarget_angle - hpos) > 0.6:
            if self.step_done:
                self.step_done = False
                hpos = hpos + 1 * (1 if trarget_angle > hpos else -1)
                sim_ok, self.sim.simxSetJointTargetPosition(self.client_id, joint, hpos*self.math.pi/180, self.sim.simx_opmode_blocking)
                self.sim.simxSynchronousTrigger(self.client_id)
                self.step_done = True

    # 1 - take/ -1 - release
    def hand_grip(self, joint, action):
        sim_ok, new_pos = self.sim.simxGetJointPosition(self.client_id, joint, self.sim.simx_opmode_buffer)
        old_pos = new_pos
        eps = 1
        while eps > 0.001:
            if self.step_done:
                self.step_done = False
                old_pos = new_pos
                self.sim.simxSetJointTargetVelocity(self.client_id, joint, -0.1*action, self.sim.simx_opmode_blocking)
                self.sim.simxSetJointForce(self.client_id, joint, 100, self.sim.simx_opmode_blocking)
                self.time.sleep(0.01)
                sim_ok, new_pos = self.sim.simxGetJointPosition(self.client_id, joint, self.sim.simx_opmode_blocking)
                print(old_pos, new_pos)
                eps = abs(old_pos - new_pos)
                if eps == 0:
                    eps = 1
                self.sim.simxSynchronousTrigger(self.client_id)
                self.step_done = True

    def waitForMovementExecuted(self, id):
        # global executedMovId
        # global stringSignalName
        while self.executedMovId!=id:
            sim_ok, s = self.sim.simxGetStringSignal(self.client_id ,self.stringSignalName, self.sim.simx_opmode_buffer)
            if sim_ok == self.sim.simx_return_ok:
                if type(s)==bytearray:
                    s=s.decode('ascii') # python2/python3 differences
                self.executedMovId=s
            self.sim.simxSynchronousTrigger(self.client_id)
        print('done')

    # Return BGR Image[width, height, id_color] with use PIL and numpy
    def vision_Sensor_Get_Image_ByPIL(self):
        sim_ok, self.resol_img, img_sim = self.sim.simxGetVisionSensorImage(self.client_id, self.v_handle_RGB, 0 ,self.sim.simx_opmode_buffer)
        if sim_ok == self.sim.simx_return_ok:
            img_b_arr = self.array.array('b', img_sim)
            img_buffer = self.I.frombuffer("RGB", (self.resol_img[0], self.resol_img[1]), bytes(img_b_arr), "raw", "BGR", 0, 1)
            img_buffer=img_buffer.transpose(self.I.FLIP_TOP_BOTTOM) # Top-Bottom flip image
            img_cv2 = self.np.asarray(img_buffer)
            return img_cv2 
        else:
            return []

    # Return BGR Image[width, height, id_color] with use numpy and cv2
    def vision_Sensor_Get_Image(self):
        sim_ok, self.resol_img, img_sim = self.sim.simxGetVisionSensorImage(self.client_id, self.v_handle_RGB, 0 ,self.sim.simx_opmode_buffer)
        if sim_ok == self.sim.simx_return_ok:
            img_buffer = self.np.array(img_sim, dtype=self.np.uint8)
            img_buffer.shape = (self.resol_img[1], self.resol_img[0],3)
            img_buffer = self.np.flipud(img_buffer) # Up-Down flip image
            img_cv2 = self.cv2.cvtColor(img_buffer, self.cv2.COLOR_RGB2BGR)
            return img_cv2
        else:
            return []

    # Return depth map[width, height] with use numpy
    def vision_Sensor_Get_Depth(self):
        sim_ok, self.len_depth, depth_sim = self.sim.simxGetVisionSensorDepthBuffer(self.client_id, self.v_handle_depth ,self.sim.simx_opmode_buffer)
        if sim_ok == self.sim.simx_return_ok:
            depth_buffer = self.np.array(depth_sim, dtype=self.np.float64)
            depth_buffer.shape = (self.resol_img[1], self.resol_img[0])
            depth_cv2 = self.np.flipud(depth_buffer) # Up-Down flip depth map
            return depth_cv2
        else:
            return []

    # Make window with BGR image
    def vision_Sensor_Display(self):
        if self.Vision_Sensor_Display_On:
            self.cv2.imshow('Vision Sensor', self.img)

    # Make window with depth map
    def vision_Sensor_Display_D(self):
        if self.Vision_Sensor_Display_On:
            self.cv2.imshow('Depth Sensor', self.depth)


    def make_test(self):
        # Set-up some movement variables:
        times=[0.000, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, 8]
        j1=[0.000, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.2, 1.3, 1.4, 1.5, 1.570]
        j2=[0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000]
        j3=[0.000, -0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7, -0.8, -0.9, -1,-1.047,-1.047,-1.047,-1.047,-1.047,-1.047]
        j4=[0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000]
        j5=[0.000, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.143, 1.143, 1.143, 1.143, 1.143]
        j6=[0.000, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.2, 1.3, 1.4, 1.5, 1.570]

        # times=[0.000,0.050,0.100,0.150,0.200,0.250,0.300,0.350,0.400,0.450,0.500,0.550,0.600,0.650,0.700,0.750,0.800,0.850,0.900,0.950,1.000,1.050,1.100,1.150,1.200,1.250,1.300,1.350,1.400,1.450,1.500,1.550,1.600,1.650,1.700,1.750,1.800,1.850]
        # j1=[0.000,0.000,0.002,0.009,0.022,0.042,0.068,0.100,0.139,0.185,0.237,0.296,0.360,0.431,0.506,0.587,0.669,0.753,0.838,0.923,1.008,1.091,1.170,1.243,1.308,1.365,1.414,1.455,1.491,1.519,1.541,1.557,1.566,1.569,1.570,1.571,1.571,1.571]
        # j2=[0.000,0.000,0.002,0.009,0.022,0.042,0.068,0.100,0.140,0.185,0.237,0.296,0.361,0.431,0.507,0.587,0.670,0.754,0.838,0.924,1.009,1.092,1.171,1.243,1.308,1.365,1.414,1.455,1.491,1.519,1.541,1.557,1.566,1.569,1.570,1.571,1.571,1.571]
        # j3=[0.000,0.000,-0.002,-0.009,-0.022,-0.042,-0.068,-0.100,-0.139,-0.185,-0.237,-0.296,-0.361,-0.433,-0.511,-0.595,-0.681,-0.767,-0.854,-0.942,-1.027,-1.107,-1.182,-1.249,-1.311,-1.365,-1.414,-1.455,-1.491,-1.519,-1.541,-1.557,-1.566,-1.569,-1.570,-1.571,-1.571,-1.571]
        # j4=[0.000,0.000,0.002,0.009,0.022,0.042,0.068,0.100,0.139,0.185,0.237,0.296,0.361,0.433,0.511,0.595,0.681,0.768,0.855,0.942,1.027,1.107,1.181,1.249,1.310,1.365,1.413,1.455,1.490,1.519,1.541,1.556,1.565,1.569,1.570,1.570,1.570,1.571]
        # j5=[0.000,0.000,0.002,0.009,0.022,0.042,0.068,0.100,0.139,0.185,0.237,0.296,0.361,0.433,0.511,0.595,0.681,0.768,0.855,0.942,1.028,1.108,1.182,1.250,1.311,1.366,1.414,1.455,1.491,1.519,1.541,1.557,1.566,1.569,1.570,1.571,1.571,1.571]
        # j6=[0.000,0.000,0.002,0.009,0.022,0.042,0.068,0.100,0.139,0.185,0.237,0.296,0.361,0.433,0.511,0.595,0.681,0.768,0.855,0.942,1.027,1.108,1.182,1.249,1.311,1.365,1.414,1.455,1.491,1.519,1.541,1.557,1.566,1.569,1.570,1.571,1.571,1.571]
        
        # times=[0.000,0.050,0.100,0.150,0.200,0.250,0.300,0.350,0.400,0.450,0.500,0.550,0.600,0.650,0.700,0.750,0.800,0.850,0.900,0.950,1.000,1.050,1.100,1.150,1.200,1.250,1.300,1.350,1.400,1.450,1.500,1.550,1.600,1.650,1.700,1.750,1.800,1.850,1.900,1.950,2.000,2.050,2.100,2.150,2.200,2.250,2.300,2.350,2.400,2.450,2.500,2.550,2.600,2.650,2.700,2.750,2.800,2.850,2.900,2.950,3.000,3.050,3.100,3.150,3.200,3.250,3.300,3.350,3.400,3.450,3.500,3.550,3.600,3.650,3.700,3.750,3.800,3.850,3.900,3.950,4.000,4.050,4.100,4.150,4.200,4.250,4.300,4.350,4.400,4.450,4.500,4.550,4.600,4.650,4.700,4.750,4.800,4.850,4.900,4.950,5.000,5.050,5.100,5.150,5.200,5.250,5.300,5.350,5.400,5.450,5.500,5.550,5.600,5.650,5.700,5.750,5.800,5.850,5.900,5.950,6.000,6.050,6.100,6.150,6.200,6.250,6.300,6.350]
        # j1=[0.000,0.000,0.002,0.009,0.022,0.042,0.068,0.100,0.139,0.185,0.237,0.296,0.360,0.431,0.506,0.587,0.669,0.753,0.838,0.923,1.008,1.091,1.170,1.243,1.308,1.365,1.414,1.455,1.491,1.519,1.541,1.557,1.566,1.564,1.556,1.543,1.524,1.498,1.465,1.426,1.380,1.328,1.270,1.205,1.136,1.065,0.994,0.922,0.849,0.777,0.705,0.632,0.560,0.487,0.415,0.342,0.270,0.197,0.125,0.053,-0.020,-0.092,-0.165,-0.237,-0.309,-0.382,-0.454,-0.527,-0.599,-0.671,-0.744,-0.816,-0.888,-0.961,-1.033,-1.106,-1.178,-1.250,-1.323,-1.394,-1.462,-1.523,-1.556,-1.595,-1.632,-1.664,-1.690,-1.709,-1.723,-1.729,-1.730,-1.723,-1.710,-1.691,-1.665,-1.632,-1.593,-1.548,-1.495,-1.437,-1.372,-1.302,-1.226,-1.146,-1.064,-0.980,-0.895,-0.810,-0.724,-0.638,-0.552,-0.469,-0.390,-0.318,-0.254,-0.199,-0.151,-0.110,-0.076,-0.048,-0.027,-0.012,-0.004,-0.001,-0.001,-0.000,-0.000,-0.000]
        # j2=[0.000,0.000,0.002,0.009,0.022,0.042,0.068,0.100,0.140,0.185,0.237,0.296,0.361,0.431,0.507,0.587,0.670,0.754,0.838,0.924,1.009,1.092,1.171,1.243,1.308,1.365,1.414,1.455,1.491,1.519,1.541,1.557,1.566,1.564,1.557,1.544,1.529,1.513,1.497,1.481,1.465,1.449,1.432,1.416,1.400,1.384,1.367,1.351,1.335,1.319,1.303,1.286,1.270,1.254,1.238,1.221,1.205,1.189,1.173,1.157,1.140,1.124,1.108,1.092,1.075,1.059,1.043,1.027,1.010,0.994,0.978,0.961,0.945,0.929,0.913,0.896,0.880,0.864,0.848,0.831,0.815,0.799,0.786,0.769,0.749,0.730,0.710,0.689,0.669,0.649,0.629,0.609,0.589,0.569,0.548,0.528,0.508,0.488,0.468,0.448,0.427,0.407,0.387,0.367,0.347,0.327,0.306,0.286,0.266,0.246,0.226,0.206,0.186,0.166,0.146,0.125,0.105,0.084,0.064,0.044,0.025,0.012,0.004,0.001,0.000,0.000,0.000,0.000]
        # j3=[0.000,0.000,-0.002,-0.009,-0.022,-0.042,-0.068,-0.100,-0.139,-0.185,-0.237,-0.296,-0.361,-0.433,-0.511,-0.595,-0.681,-0.767,-0.854,-0.942,-1.027,-1.107,-1.182,-1.249,-1.311,-1.365,-1.414,-1.455,-1.491,-1.519,-1.541,-1.557,-1.566,-1.564,-1.556,-1.543,-1.524,-1.498,-1.465,-1.426,-1.381,-1.328,-1.270,-1.205,-1.133,-1.055,-0.971,-0.885,-0.798,-0.711,-0.624,-0.537,-0.450,-0.362,-0.275,-0.188,-0.101,-0.013,0.074,0.161,0.249,0.336,0.423,0.510,0.598,0.685,0.772,0.859,0.947,1.032,1.112,1.186,1.253,1.314,1.369,1.416,1.458,1.492,1.521,1.542,1.557,1.566,1.564,1.557,1.544,1.524,1.498,1.466,1.427,1.383,1.338,1.293,1.247,1.201,1.155,1.110,1.064,1.018,0.972,0.926,0.881,0.835,0.789,0.743,0.697,0.652,0.606,0.560,0.514,0.468,0.423,0.377,0.331,0.285,0.239,0.194,0.149,0.109,0.076,0.048,0.027,0.012,0.004,0.002,0.001,0.000,0.000,0.000]
        # j4=[0.000,0.000,0.002,0.009,0.022,0.042,0.068,0.100,0.139,0.185,0.237,0.296,0.361,0.433,0.511,0.595,0.681,0.768,0.855,0.942,1.027,1.107,1.181,1.249,1.310,1.365,1.413,1.455,1.490,1.519,1.541,1.556,1.565,1.567,1.574,1.587,1.603,1.619,1.636,1.653,1.670,1.686,1.703,1.720,1.737,1.754,1.771,1.788,1.805,1.822,1.839,1.856,1.873,1.890,1.907,1.923,1.940,1.957,1.974,1.991,2.008,2.025,2.042,2.059,2.076,2.093,2.110,2.127,2.144,2.161,2.178,2.195,2.212,2.229,2.246,2.263,2.280,2.297,2.314,2.331,2.344,2.352,2.350,2.343,2.330,2.310,2.284,2.252,2.212,2.167,2.115,2.056,1.991,1.919,1.841,1.760,1.679,1.597,1.515,1.433,1.350,1.268,1.186,1.104,1.022,0.940,0.858,0.776,0.694,0.612,0.530,0.452,0.379,0.312,0.252,0.198,0.151,0.110,0.076,0.048,0.027,0.012,0.004,0.002,0.001,0.000,0.000,0.000]
        # j5=[0.000,0.000,0.002,0.009,0.022,0.042,0.068,0.100,0.139,0.185,0.237,0.296,0.361,0.433,0.511,0.595,0.681,0.768,0.855,0.942,1.028,1.108,1.182,1.250,1.311,1.366,1.414,1.455,1.491,1.519,1.541,1.557,1.566,1.566,1.566,1.566,1.566,1.566,1.566,1.566,1.566,1.567,1.567,1.567,1.567,1.567,1.567,1.567,1.567,1.567,1.568,1.568,1.568,1.568,1.568,1.568,1.568,1.568,1.568,1.568,1.568,1.569,1.569,1.569,1.569,1.569,1.569,1.569,1.569,1.569,1.569,1.570,1.570,1.570,1.570,1.570,1.570,1.570,1.570,1.570,1.571,1.571,1.568,1.561,1.548,1.529,1.503,1.470,1.431,1.388,1.342,1.297,1.251,1.205,1.159,1.113,1.067,1.021,0.975,0.929,0.883,0.837,0.791,0.745,0.699,0.653,0.607,0.561,0.516,0.470,0.424,0.378,0.332,0.286,0.240,0.194,0.149,0.109,0.076,0.048,0.027,0.012,0.004,0.002,0.001,0.000,0.000,0.000]
        # j6=[0.000,0.000,0.002,0.009,0.022,0.042,0.068,0.100,0.139,0.185,0.237,0.296,0.361,0.433,0.511,0.595,0.681,0.768,0.855,0.942,1.027,1.108,1.182,1.249,1.311,1.365,1.414,1.455,1.491,1.519,1.541,1.557,1.566,1.566,1.566,1.566,1.566,1.566,1.566,1.566,1.566,1.567,1.567,1.567,1.567,1.567,1.567,1.567,1.567,1.567,1.567,1.568,1.568,1.568,1.568,1.568,1.568,1.568,1.568,1.568,1.569,1.569,1.569,1.569,1.569,1.569,1.569,1.569,1.569,1.569,1.570,1.570,1.570,1.570,1.570,1.570,1.570,1.570,1.570,1.571,1.571,1.571,1.569,1.561,1.548,1.529,1.503,1.470,1.431,1.388,1.343,1.297,1.251,1.205,1.159,1.113,1.067,1.021,0.975,0.929,0.883,0.837,0.791,0.745,0.699,0.653,0.607,0.561,0.515,0.470,0.424,0.378,0.332,0.286,0.240,0.194,0.149,0.109,0.076,0.048,0.027,0.012,0.004,0.002,0.001,0.000,0.000,0.000]
        
        # Wait until ready:
        self.waitForMovementExecuted('ready') 

        # Send the movement sequence:
        targetConfig=[90*self.math.pi/180,90*self.math.pi/180,-90*self.math.pi/180,90*self.math.pi/180,90*self.math.pi/180,90*self.math.pi/180]
        movementData={"id":"movSeq1","type":"pts","times":times,"j1":j1,"j2":j2,"j3":j3,"j4":j4,"j5":j5,"j6":j6}
        packedMovementData=self.msgpack.packb(movementData)
        self.sim.simxCallScriptFunction(self.client_id, self.targetArm, self.sim.sim_scripttype_childscript, 'legacyRapiMovementDataFunction', [] ,[], [], packedMovementData, self.sim.simx_opmode_oneshot)

        # Execute movement sequence:
        self.sim.simxCallScriptFunction(self.client_id, self.targetArm, self.sim.sim_scripttype_childscript, 'legacyRapiExecuteMovement', [], [], [], 'movSeq1', self.sim.simx_opmode_oneshot)
        # Wait until above movement sequence finished executing:
        self.waitForMovementExecuted('movSeq1')

if __name__ == '__main__':
    kinect = Motion_sim() # Init object
    kinect.sim_Start()   # Start simulation
    kinect.set_pos(kinect.handles_KUKA[1], 60)
    kinect.hand_grip(kinect.handle_grip, -1)
    sim_ok, cube = kinect.sim.simxGetObjectHandle(kinect.client_id, 'Cuboid', kinect.sim.simx_opmode_blocking)
    kinect.set_pos(kinect.handles_KUKA[0], 90)
    kinect.set_pos(kinect.handles_KUKA[6], 90)
    kinect.set_pos(kinect.handles_KUKA[5], 65.5)
    kinect.set_pos(kinect.handles_KUKA[3], -60)
    kinect.set_pos(kinect.handles_KUKA[2], 0)
    kinect.set_pos(kinect.handles_KUKA[1], 60)
    kinect.time.sleep(1)
    kinect.hand_grip(kinect.handle_grip, 1)
    kinect.sim.simxSetObjectParent(kinect.client_id, cube, kinect.handle_attach_point, True, kinect.sim.simx_opmode_blocking)
    kinect.set_pos(kinect.handles_KUKA[3], 0)
    kinect.set_pos(kinect.handles_KUKA[0], 0)
    kinect.set_pos(kinect.handles_KUKA[6], 0)
    kinect.set_pos(kinect.handles_KUKA[5], 0)
    kinect.set_pos(kinect.handles_KUKA[2], 0)
    kinect.set_pos(kinect.handles_KUKA[1], 0)

    # kinect.make_test()
    kinect.hand_grip(kinect.handle_grip, 1)
    # Show image and depth map cycle
    while (kinect.sim.simxGetConnectionId(kinect.client_id) != -1):
        kinect.img = kinect.vision_Sensor_Get_Image_ByPIL()
        kinect.depth = kinect.vision_Sensor_Get_Depth()
        if kinect.img != []:
            kinect.vision_Sensor_Display()
        else:
            print('null img')
        if kinect.depth != []:
            kinect.vision_Sensor_Display_D()
        else:
            print('null depth')
        if kinect.cv2.waitKey(1) & 0xff == ord('q'):
            break
        kinect.sim.simxSynchronousTrigger(kinect.client_id)
    kinect.sim.simxStopSimulation(kinect.client_id, kinect.sim.simx_opmode_blocking)