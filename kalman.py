import math
import numpy as np
import time

# Kalman class used for the Kalman filter

class Kalman:
    
    def __init__(self,thymio):
        
        # Transition state matrix
        self.A = np.matrix([[1,0,0],[0,1,0],[0,0,1]],dtype= 'float')

        # Transition input matrix
        self.B = np.matrix([[1,0,0],[0,1,0],[0,0,1]],dtype= 'float')

        # Observation matrix. Current estimated position
        # need to get this from navigation or control
        self.E = np.matrix([[thymio.x_cam],[thymio.y_cam],[thymio.theta_cam]],dtype= 'float')

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
        self.b = 80

        # Delta time 
        self.lastKalman = time.time_ns()/10e8

        # Speed from pwm to mm/s
        # Depends on which thymio
        self.conv = 0.43


    def kalman_prediction(self,thymio,L_speed,R_speed):
        
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
        self.E = np.dot(self.A, self.E) + np.dot(self.B, self.U)*deltaT
        
        # Uncertainty due to the motors B⋅Uvar⋅B'+I
        Q = np.dot(self.B, np.dot(self.U_var_m,self.B.T)) + np.eye(3)
        
        # Update the variance of the system APA'+Q (slide 44)
        self.P = np.dot(np.dot(self.A,self.P),np.transpose(self.A))+Q
        
        # Update the state with the predicted x, y and theta
        thymio.x_est = self.E[0].item()
        thymio.y_est = self.E[1].item()
        thymio.theta_est = self.E[2].item()
        
        # Keep speeds of the robot to compute the next prediction
        self.U = np.matrix([[L_speed],[R_speed]],dtype= 'float')
        
        # Update the time of the last kalman done to find deltaT
        self.lastKalman = time.time_ns()/10e8

    def kalman_update(self,thymio):

        # update the state of the robot and  the variances of the system 
        # with camera measurements
        
        # Measured state matrix
        Z = np.matrix([[thymio.x_cam],[thymio.y_cam],[thymio.theta_cam]],dtype= 'float')
        
        # Update the state of the Kalman with the state of the robot
        # These need to be defined in navigation or control 
        self.E[0] = thymio.x_est 
        self.E[1] = thymio.y_est 
        self.E[2] = thymio.theta_est 
        
        # Kalman gain (slide 48)
        K_den = np.linalg.inv(np.dot(self.H,np.dot(self.P,np.transpose(self.H))) + self.R)
        K_num = np.dot(self.P,np.transpose(self.H))
        K = np.dot(K_num,K_den)        
        
        # Correction of the state with the Kalman gain and the measurements 
        # E=E+K⋅(Z−H⋅E)
        # Z - HE is the difference between the measured and the predicted state
        self.E = self.E + np.dot(K,(Z-np.dot(self.H,self.E)))
        
        # Update of the variance of the system (slide 48)
        I = np.eye(3)
        self.P = np.dot((I-np.dot(K,self.H)),self.P)
        
        # Update the state with the corrected x, y and theta
        thymio.x_est = self.E[0].item()
        thymio.y_est = self.E[1].item()
        thymio.theta_est = self.E[2].item()
        
        # Update the time of the last kalman done to find deltaT
        self.lastKalman = time.time_ns()/10e8

        return False