import math
import numpy as np
import time

# Kalman class used for the Kalman filter

# PSEUDOCODE for the main control loop:
#     Always run the prediction step
#     kalman.kalman_prediction(thymio, L_speed, R_speed)  

#     if camera_available():  If the camera sees the robot
#         thymio.x_cam, thymio.y_cam, thymio.theta_cam = get_camera_measurements()
#         kalman.kalman_update(thymio)  Update the state with camera measurements

#     Use the updated/estimated state for navigation
#     navigate_to_goal(thymio.x_est, thymio.y_est, thymio.theta_est)

class Kalman:
    
    def __init__(self):        

        # Transition state matrix
        self.A = np.matrix([[1,0,0],[0,1,0],[0,0,1]],dtype= 'float')

        # Observation matrix. Current estimated position (Corresponds to mu in the slides)
        self.E = np.matrix([[0],[0],[0]],dtype= 'float')

        # Transition input matrix. Jacobian of the motion model
        self.G = np.matrix([[1,0,0],[0,1,0],[0,0,1]],dtype= 'float')

        # Jacobian of the measurement model. Measurement transformation matrix
        # Since we measure directly x, y, theta, H is simply an identity matrix
        self.H = np.matrix([[1,0,0],[0,1,0],[0,0,1]],dtype= 'float')

        # Covariance matrix (Corresponds to SIGMA in the slides)
        # Higher values -> more uncertain
        self.P = np.matrix([[10,0,0],[0,10,0],[0,0,1]],dtype= 'float')

        # Variances of measurement (x, y, theta)
        # Arbitrary values
        q1 = 1 
        q2 = 1
        q3 = np.pi / 180 # 1 rad
        self.Q = np.matrix([[q1,0,0],[0,q2,0],[0,0,q3]],dtype= 'float')

        # Motor speeds
        self.U = np.matrix([[0],[0]],dtype= 'float')

        # Variances of motors (Needed to compute Q)
        # Depends on which thymio
        u1 = 22
        u2 = 22
        self.U_var = np.diag([u1,u2])

        # Distance between the two wheels
        self.d = 93.5

        # Delta time 
        self.lastKalman = time.time_ns()/1e9

        # Speed from pwm to mm/s
        # Depends on which thymio
        self.c = 0.3726

        # Adjustment for the thymio's wheels differences
        # Depends on which thymio
        self.adj = 1.1

    def initialize_position(self, x, y, theta):
        self.E[0, 0] = x
        self.E[1, 0] = y
        self.E[2, 0] = theta
        self.set_lastKalman_time()

    def get_state(self):
        return float(self.E[0, 0]), float(self.E[1, 0]), float(self.E[2, 0])

    def set_lastKalman_time(self):
        self.lastKalman = time.time_ns()/1e9


    def kalman_prediction(self, L_speed, R_speed, dt):
        
        # INPUT: (motors) L_speed, R_speed

        # OUTPUT: None

        # Predicts the state of the robot based on the odometry and updates the 
        # variances of the system.
        
        # Time between the last update/prediction and this time
        #deltaT = time.time_ns()/1e9 - self.lastKalman + 0.2
        #deltaT = deltaT/1e9 # Convert to seconds

        # Update speeds of the robot
        self.U = np.matrix([[L_speed],[R_speed]],dtype= 'float')

        # Jacobian of motion model corresponding to:
        # v = (c⋅Rspeed + c⋅Lspeed)/2
        # which gives
        # Δx = (c/2)⋅(Rspeed+Lspeed)⋅cos(θ)⋅Δt
        # Δθ = ((c⋅Rspeed​−c⋅Lspeed​​)/b)⋅Δt
        self.G = np.matrix([[math.sin(self.E[2])*self.c/self.adj/2, math.sin(self.E[2])*self.c/2],
                            [math.cos(self.E[2])*self.c/self.adj/2, math.cos(self.E[2])*self.c/2],
                            [self.c/self.adj/self.d, -1*self.c/self.d]],dtype= 'float')

        # Predicted state of the robot AE+GU⋅Δt (slide 41)
        self.E = self.A @ self.E + self.G @ self.U*dt
        self.E[2] = self.E[2] % (2*np.pi)

        # Uncertainty due to the motors G⋅Uvar⋅G'+I
        R = self.G @ self.U_var @ self.G.T + 0.1 * np.eye(3)
        
        # Update the variance of the system APA'+R (slide 44)
        self.P = self.A @ self.P @ self.A.T + R
        
        # Update the time of the last kalman done to find deltaT
        self.lastKalman = time.time_ns()/1e9

    def kalman_update(self, measurement):

        # INPUT: measurement array
        
        # OUTPUT: None, modifies measurement

        # update the state of the robot and  the variances of the system 
        # with camera measurements
        
        # Measured state matrix
        Z = np.matrix([[measurement[0]],[measurement[1]],[measurement[2]]],dtype= 'float')
        
        # Kalman gain (slide 48)
        K1 = np.linalg.inv(np.dot(self.H,np.dot(self.P,np.transpose(self.H))) + self.Q)
        K2 = np.dot(self.P,np.transpose(self.H))
        K = np.dot(K2,K1)        
        
        # Correction of the state with the Kalman gain and the measurements 
        # E=E+K⋅(Z−H⋅E)
        # Z - HE is the difference between the measured and the predicted state
        self.E = self.E + np.dot(K,(Z-np.dot(self.H,self.E)))
        self.E[2] = self.E[2] % (2*np.pi)
        
        # Update of the variance of the system (slide 48)
        I = np.eye(3)
        self.P = np.dot((I-np.dot(K,self.H)),self.P)
        
        # Update the time of the last kalman done to find deltaT
        self.lastKalman = time.time_ns()/1e9