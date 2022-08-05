from threading import local
import rospy
import numpy as np
import local_cartesian
import math as m
import tf2_ros
import tf_conversions
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, TransformStamped
import time
from geometry_msgs.msg import PoseStamped, TransformStamped

class NDT:
    def __init__(self):
        self.ndt_pose=np.zeros(2, dtype=float)
        self.ndt_cov = np.eye(2)
        self.ndt = 0
        self.old_time = time.time()
        
    def ndt_stat_callback(self, data):
        if(data.score > 0.10 and data.score < 0):
            arr = [20, 20]
            self.ndt_cov = np.eye(2)*arr
        else:
            arr = [0.001, 0.001]
            self.ndt_cov = np.eye(2)*arr
    
    def ndt_pose_local_callback(self, data):
        self.ndt_pose[0]=data.pose.position.x
        self.ndt_pose[1]=data.pose.position.y
        self.ndt = data
        self.old_time = time.time()
        
    def get_cov(self):
        return self.ndt_cov

    def get_ndt_local_pose(self):
        return self.ndt_pose




class GNSS:
    def __init__(self):
        self.lla=np.zeros(2, dtype=float)
        self.lla_cov = np.eye(2)
        self.__initLLA = np.empty((0,3), dtype=float)
        self.time = 0
        
    def callback(self, data):
        self.lla[0]=data.latitude
        self.lla[1]=data.longitude
        arr = [data.position_covariance[0],data.position_covariance[4]]
        self.lla_cov = (np.eye(2)*arr)
 
        
        #print("gnss data is comming")
            
    def initialization(self):
        Stack = np.empty((0,2), dtype=float)
        for enum in range(0,3):
            Stack = np.append(Stack, [self.lla],axis=0)
        self.__initLLA = np.mean(Stack,axis=0)
        
        print("Initialization has been complete.")
        
    def get_init(self):
        return self.__initLLA
    
    def get_cov(self):
        return self.lla_cov    
    
    def local_Cartesian(self):
        local_data = local_cartesian.local_cartesian_forward(self.__initLLA, self.lla)
        return local_data
    
    def local_Cartesian2(self,data):
       
        Data = local_cartesian.local_cartesian_forward(self.__initLLA, data)
        return Data
    
    def l2g_Cartesian(self, data):
        self.global_data = local_cartesian.local_cartesian_reverse(self.__initLLA,data)
        #print(self.global_data)
        return self.global_data
    
    
class IMU:
    def __init__(self): 
        self.ori = np.empty(3, dtype=float)
        self.gyr = np.empty(3, dtype=float)
        self.__initori= np.empty(3, dtype=float)
        self.prev_time = 0
        self.prev_ori = 0
      
        
    def callback(self,data):
    
        quaternion = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        self.ori[0] = tf_conversions.transformations.euler_from_quaternion(quaternion)[0]
        self.ori[1] = tf_conversions.transformations.euler_from_quaternion(quaternion)[1]
        self.ori[2] = tf_conversions.transformations.euler_from_quaternion(quaternion)[2]
        self.gyr[0] = data.angular_velocity.x
        self.gyr[1] = data.angular_velocity.y
        self.gyr[2] = data.angular_velocity.z
        
    def rotation_matrix(self):
        angle = self.correction(-1*self.__initori[2]-m.radians(27.0))
        Rot = np.array([[m.cos(angle),m.sin(angle)*-1.0,0.0], [m.sin(angle),m.cos(angle),0.0],[0.0, 0.0, 1.0]])
        
        return Rot
        
        
    def initialization(self):
        Stack = np.empty((0,3), dtype=float)
        for _ in range(0,10):
            Stack = np.append(Stack, [self.ori],axis=0)
        self.__initori = np.mean(Stack,axis=0)
        self.prev_time = time.time()
        self.prev_ori = self.__initori[2]
        print("Initialization has been complete.")
        
    def correction(self, angle):
        if angle < -m.pi:
            data = angle + 2*m.pi 
        elif angle > m.pi:
            data = angle - 2*m.pi
        else:
            data = angle        
        return data

    def get_data(self):
        return self.correction(self.ori[2]-self.__initori[2])*180/m.pi
    
    def get_yawrate(self):
        # DT = time.time() - self.prev_time
        # self.prev_time = time.time()
        # DYaw = self.ori[2]- self.prev_ori
        # self.prev_ori = self.ori[2]
        # yawrate = DYaw / DT
        return self.gyr[2]
        