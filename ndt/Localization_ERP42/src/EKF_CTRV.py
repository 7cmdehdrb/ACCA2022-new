#! /usr/bin/env python

import math
import matplotlib.pyplot as plt
import numpy as np
import rospy
import time


class EKF:
    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(EKF, cls).__new__(cls)
        return cls.instance
    
    def __init__(self, variance_x=0.05, variance_y = 0.05, variance_vel = 0.5, variance_yaw = 0.5, variance_gyr = 0.5):
        self.Q = np.diag([
                        variance_x, # variance of location on x-axis
                        variance_y, # variance of location on y-axis
                        variance_vel, # variance of velocity
                        np.deg2rad(variance_yaw), # variance of yaw angle
                        np.deg2rad(variance_gyr) # variance of yaw rate
                    ])**2 # preict state covariance

        self.v = 0.0 # [m/s]
        self.yawrate = 0.1 # [rad/s]
        self.xEst = np.zeros((5,1), dtype=float)
        self.PEst = np.eye(5, dtype=float)
        self.DT = 0.1
        self.start = 0
       
    
    def callback(self, data):
        self.v = data.speed
        #print("speed:", data.speed)
        
    
    def calc_input(self, yawrate):
        if yawrate == 0.0:
            yawrate = 0.1
        u = np.array([[self.v],[yawrate]])

        return u

    def motion_model(self,x, u, yaw):
        x[3,0] = yaw
        
        F = np.array([
            [1.0, 0, 0, 0, 0],
            [0, 1.0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 1.0, 0],
            [0, 0, 0, 0, 0]])
        B = np.array([[(math.sin(x[3,0]+u[1,0]*self.DT)-math.sin(x[3,0]))/u[1,0], 0],
                    [(-math.cos(x[3,0]+u[1,0]*self.DT)+math.cos(x[3,0]))/u[1,0], 0],
                    [1.0, 0.0],
                    [0.0, self.DT],
                    [0.0, 1.0]])
        x = np.matmul(F, x) + np.matmul(B, u)
        return x

    def observation_model(self,x):
        H = np.array([[1,0,0,0, 0],
                    [0,1,0,0, 0]])
        z = np.matmul(H , x)
        return z

    def jacob_f(self, x, u):
        yaw = x[3,0]
        v = u[0,0]
        yawrate = u[1,0]
        jF = np.array([[1.0, 0.0, (math.sin(yaw+yawrate*self.DT)-math.sin(yaw))/yawrate,v/yawrate*(math.cos(yaw+yawrate*self.DT)-math.cos(yaw)),v*math.cos(yaw+yawrate*self.DT)/yawrate-v*(math.sin(yaw+yawrate*self.DT)-math.sin(yaw))/(yawrate**2)],
                    [0.0, 1.0, (-math.cos(yaw+yawrate*self.DT)+math.cos(yaw))/yawrate,v/yawrate*(-math.sin(yaw+yawrate*self.DT)+math.sin(yaw)),v*math.sin(yaw+yawrate*self.DT)/yawrate-v*(-math.cos(yaw+yawrate*self.DT)+math.cos(yaw))/(yawrate**2)],
                    [0.0, 0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0, self.DT],
                    [0.0, 0.0, 0.0, 0.0, 1.0]])
        return jF

    def jacob_h(self):
        # Jacobian of Observation Model
        jH = np.array([[1, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0]])
        return jH

    def ekf_prediction(self, IMU):
        # Predict               
        u = self.calc_input(IMU.get_yawrate())     
        self.xPred = self.motion_model(self.xEst, u, IMU.get_data())
        jF = self.jacob_f(self.xEst, u)
        self.PPred = np.matmul(np.matmul(jF , self.PEst) , jF.T) + self.Q
        
    def update(self, z_gnss,z_ndt, R_gnss, R_ndt, update ):
        jH = self.jacob_h()
        zPred = self.observation_model(self.xPred)
        if update: 
            y_gnss = z_gnss-zPred
            S_gnss = np.matmul(np.matmul(jH,self.PPred) , jH.T) + R_gnss
            K_gnss = np.matmul(np.matmul(self.PPred, jH.T) , np.linalg.inv(S_gnss))
            self.xEst = self.xPred + np.matmul(K_gnss , y_gnss)
            self.PEst = np.matmul((np.eye(len(self.xEst))- np.matmul(K_gnss,jH)) , self.PPred)
            
        else:
            y_gnss = z_gnss-zPred
            y_ndt = z_ndt-zPred
            S_gnss = np.matmul(np.matmul(jH,self.PPred) , jH.T) + R_gnss
            S_ndt = np.matmul(np.matmul(jH,self.PPred) , jH.T) + R_ndt
            K_gnss = np.matmul(np.matmul(self.PPred, jH.T) , np.linalg.inv(S_gnss))
            K_ndt = np.matmul(np.matmul(self.PPred, jH.T) , np.linalg.inv(S_ndt))
            self.xEst = self.xPred + np.matmul(K_gnss , y_gnss) + np.matmul(K_ndt , y_ndt)
            self.PEst = np.matmul((np.eye(len(self.xEst))- (np.matmul(K_gnss,jH)+np.matmul(K_ndt,jH))/2) , self.PPred)
        return self.xEst

   

if __name__ == '__main__':
    ekf = EKF()
    
    


