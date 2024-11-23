import math
import numpy as np
import time

# Kalman class used for the Kalman filter

# NEEDS A THYMIO CLASS, WHICH MEMORIZES x_cam, y_cam, theta_cam, x_est, y_est, theta_est
# THE CLASS NEEDS TO BE INIZIALIZED BEFORE THE KALMAN FILTER

# PSEUDOCODE for the main control loop:
#     Always run the prediction step
#     kalman.kalman_prediction(thymio, L_speed, R_speed)  

#     if camera_available():  If the camera sees the robot
#         thymio.x_cam, thymio.y_cam, thymio.theta_cam = get_camera_measurements()
#         kalman.kalman_update(thymio)  Update the state with camera measurements

#     # Use the updated/estimated state for navigation
#     navigate_to_goal(thymio.x_est, thymio.y_est, thymio.theta_est)

class Kalman:
    
    def __init__(self):        
        # INPUT: thymio instance, a class needed to memorize the state of the robot
        #        this class contains (camera) x_cam, y_cam, theta_cam, (estimations) x_est, y_est, theta_est

        # OUTPUT: None. Modifies the thymio instance

        # Transition state matrix
        self.A = np.matrix([[1,0,0],[0,1,0],[0,0,1]],dtype= 'float')

        # Transition input matrix
        self.B = np.matrix([[1,0,0],[0,1,0],[0,0,1]],dtype= 'float')

        # Observation matrix. Current estimated position
        self.E = np.matrix([[0],[0],[0]],dtype= 'float')

        # Motor speeds
        self.U = np.matrix([[0],[0]],dtype= 'float')

        # Covariance matrix (Corresponds to SIGMA in the slides)
        # Higher values -> more uncertain
        self.P = np.matrix([[10,0,0],[0,10,0],[0,0,1]],dtype= 'float')

        # Measurement matrix 
        self.H = np.matrix([[1,0,0],[0,1,0],[0,0,1]],dtype= 'float')

        # Variances of motors (corresponds to Q in the slides)
        # Depends on which thymio
        u1 = 19
        u2 = 19
        self.U_var = np.diag([u1,u2])

        # Variances of measurement (x, y, theta)
        # Arbitrary values
        r1 = 1 
        r2 = 1
        r3 = np.pi/180 # 1 rad
        self.R = np.matrix([[r1,0,0],[0,r2,0],[0,0,r3]],dtype= 'float')

        # Distance between the two wheels
        self.b = 93

        # Delta time 
        self.lastKalman = time.time_ns()/10e8

        # Speed from pwm to mm/s
        # Depends on which thymio
        self.c = 0.43

    def initialize_position(self, x, y, theta):
            self.E[0] = x
            self.E[1] = y
            self.E[2] = theta

    def get_state(self):
        return self.E[0], self.E[1], self.E[2]

    def set_lastKalman_time(self):
        self.lastKalman = time.time_ns()/10e8


    def kalman_prediction(self, L_speed, R_speed):
        
        # INPUT: (motors) L_speed, R_speed

        # OUTPUT: None. Modifies the thymio instance

        # Predicts the state of the robot based on the odometry and updates the 
        # variances of the system.
        
        # Time between the last update/prediction and this time
        deltaT = time.time_ns()/10e8 - self.lastKalman

        # Transition matrix of the input corresponding to:
        # v = (c⋅Rspeed + c⋅Lspeed)/2
        # which gives
        # Δx = (c/2)⋅(Rspeed+Lspeed)⋅cos(θ)⋅Δt
        # Δθ = ((c⋅Rspeed​−c⋅Lspeed​​)/b)⋅Δt
        self.B = np.matrix([[0.5*math.cos(self.E[2])*self.c,0.5*math.cos(self.E[2])*self.c],
                            [0.5*math.sin(self.E[2])*self.c,0.5*math.sin(self.E[2])*self.c],
                            [1/self.b*self.c, -1/self.b*self.c]],dtype= 'float')
        
        # Predicted state of the robot (AE+BU)⋅Δt
        self.E = (np.dot(self.A, self.E) + np.dot(self.B, self.U))*deltaT
        
        # Uncertainty due to the motors B⋅Uvar⋅B'+I
        Q = np.dot(self.B, np.dot(self.U_var,self.B.T)) + np.eye(3)
        
        # Update the variance of the system APA'+Q (slide 44)
        self.P = np.dot(np.dot(self.A,self.P),np.transpose(self.A))+Q
        
        # Keep speeds of the robot to compute the next prediction
        self.U = np.matrix([[L_speed],[R_speed]],dtype= 'float')
        
        # Update the time of the last kalman done to find deltaT
        self.lastKalman = time.time_ns()/10e8

    def kalman_update(self, measurement):

        # INPUT: thymio instance
        
        # OUTPUT: None. Modifies the thymio instance

        # update the state of the robot and  the variances of the system 
        # with camera measurements
        
        # Measured state matrix
        Z = np.matrix([[measurement[0]],[measurement[1]],[measurement[2]]],dtype= 'float')
        
        # Kalman gain (slide 48)
        K1 = np.linalg.inv(np.dot(self.H,np.dot(self.P,np.transpose(self.H))) + self.R)
        K2 = np.dot(self.P,np.transpose(self.H))
        K = np.dot(K2,K1)        
        
        # Correction of the state with the Kalman gain and the measurements 
        # E=E+K⋅(Z−H⋅E)
        # Z - HE is the difference between the measured and the predicted state
        self.E = self.E + np.dot(K,(Z-np.dot(self.H,self.E)))
        
        # Update of the variance of the system (slide 48)
        I = np.eye(3)
        self.P = np.dot((I-np.dot(K,self.H)),self.P)
        
        # Update the time of the last kalman done to find deltaT
        self.lastKalman = time.time_ns()/10e8