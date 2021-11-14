# -*- coding: utf-8 -*-
"""
Created on Sun Nov 14 13:35:35 2021

@author: Salim
"""
import math
import numpy as np
import time
from numpy.linalg import inv

"""
PLEASE READ ME: 
    This code is divided into two sections: 
        1.An "adapter" that allows for the import of data
        2.The original, unchanged, code 

    Through this approach, I wish to decouple the EKF code from ROS. 
    The data will be fed into the EKF from csv files.
    This should make characterization and testing much easier. 

"""

#ADAPTER CODE 
#these fields are set to 0 for now 
t265_x = 0
t265_y = 0
t265_yaw = 0 

gps_easting = 0
gps_northing = 0
um7_yaw = 0


#ORIGINAL UNMODIFIED CODE  
class RobotEKF():
    # initializes motion model with standard dev. of velocity and time step
    def __init__(self, state_init, std_vel, std_rot): 
        # create a three-dimensional state with 3 measurements (for x, y and theta; yaw) => state_init
        self.std_vel = std_vel # standard dev. of velocity and rotation
        self.std_rot = std_rot
        
        self.R_cam = np.array([[0.4,0,0], # covariance matrix for measurement noise of the tracking camera
                             [0,0.4,0],
                             [0,0,0.9]])
        
        self.R_gpIMU = np.array([[1.5, 0, 0], # covariance matrix for measurement noise of the gps and IMU readings
                                 [0, 1.5, 0],
                                 [0, 0, 0.4]])

        self.Q = np.array([[self.std_vel**2,0,0], # covariance for process noise (i.e. states)
                           [0,self.std_vel**2,0],
                           [0,0,self.std_rot**2]])
        self.P = self.Q
        
        self.state_est = state_init # initialize estimated state with parameter
        self.time_prev = time.time() # current time
        
    def get_F(self, theta_prev, dt, v): 
        """
        fxu = Matrix([[x + d*sympy.sin(theta)],
                        [y + d*sympy.cos(theta)],
                        [theta +d*omega]])
        return = fxu.jacobian(Matrix([x, y, theta])) 
        """
        # jacobian of get_predict_state ^
        F = np.array([[1,0,-dt*v*math.sin(theta_prev)],
                         [0,1,dt*v*math.cos(theta_prev)],
                         [0,0,1]])
        
        return F
    
    def get_H(self): # jacobian of sensor/measurement model (identity since they are exact)
        H = np.array([[1, 0, 0],
               [0, 1, 0],
               [0, 0, 1]])
        
        return H
    
    def get_predict_state(self, x_prev, y_prev, theta_prev, dt, v, omega): # motion model
        #Omega = angular velocity
        #v = forward velocity
        state_pred = np.array([x_prev + dt*v*math.cos(theta_prev),
                        y_prev + dt*v*math.sin(theta_prev),
                        theta_prev + dt*omega])
        
        return state_pred
    
    # sensor readings (GPS, tracking cam and IMU; x, y, theta)
    # for the first update step
    def get_cam_measurement(self):
        z = np.array([[t265_x],
             [t265_y],
             [t265_yaw]])
        return z

    # for the second update step
    def get_gpsIMU_measurement(self):
        z = np.array([[gps_easting],
                     [gps_northing],
                     [um7_yaw]])
        return z

    def predict_update(self, u=[0,0]):
        #Predict step
        dt = time.time() - self.time_prev # calculate time step
        self.time_prev = time.time() # resets current time
        state_pred = self.get_predict_state(x_prev = self.state_est[0], y_prev = self.state_est[1], theta_prev = self.state_est[2], dt = dt, v = u[0], omega = u[1]) # predicted state estimate

        F = self.get_F(theta_prev = self.state_est[2], dt = dt, v = u[0]) # state transition matrix
        H = self.get_H() # observation matrix
        
        P_pred = np.matmul(np.matmul(F,self.P),F.T) + self.Q # predicted covariance estimate (used for first update)

        ################################
        #First update step
        first_state_meas = self.get_cam_measurement() # get camera readings
        first_y_res = first_state_meas - np.matmul(H,state_pred) # first measurement residual (for cam)
        
        firstS_k = np.matmul(np.matmul(H,P_pred),H.T) + self.R_cam # first residual covariance
        firstS_k = firstS_k.astype('float')
        firstK_k = np.matmul(np.matmul(P_pred,H.T),inv(firstS_k)) # initial Kalman gain
        
        init_state_est = state_pred + np.matmul(firstK_k, first_y_res) # first updated state estimate (rover_ekf.x)
        init_P = np.matmul((np.eye(3) - np.matmul(firstK_k, H)),P_pred) # first updated covariance estimate

        ################################
        #Second (and final) update step
        second_state_meas = self.get_gpsIMU_measurement() # get gps and imu readings
        second_y_res = second_state_meas - np.matmul(H, init_state_est) # second measurement resid. (takes prev. state estimate)

        secondS_k = np.matmul(np.matmul(H, init_P), H.T) + self.R_gpIMU # uses first covariance estimate init_P ^^^
        secondS_k = secondS_k.astype('float')
        secondK_k = np.matmul(np.matmul(init_P,H.T),inv(secondS_k)) # final Kalman gain

        self.state_est = init_state_est + np.matmul(secondK_k, second_y_res) # updated + final state estimate
        self.P = np.matmul((np.eye(3) - np.matmul(secondK_k, H)), init_P) # updated + final covariance estimate
