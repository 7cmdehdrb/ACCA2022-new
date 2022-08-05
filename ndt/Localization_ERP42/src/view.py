import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
import tf2_ros 
import math as m
import tf_conversions
from tf2_geometry_msgs import PoseStamped



class Visualization:
    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(Visualization, cls).__new__(cls)
        return cls.instance
    
    def __init__(self):
        self.ekf_path_pub = rospy.Publisher("/path_ekf", Path, queue_size=10)
        self.gnss_path_pub = rospy.Publisher("/path_gnss", Path, queue_size=10)
        self.ndt_path_pub = rospy.Publisher("/path_ndt", Path, queue_size = 10)
        self.state_pub = rospy.Publisher("/current_pose", PoseStamped, queue_size = 1)
        self.ekf_pub = rospy.Publisher("/ekf_pose", PoseStamped, queue_size = 1)
        self.gps_pub = rospy.Publisher("gnss_pose", PoseStamped, queue_size=1)
        self.Est_state = PoseStamped()
        self.GNSS_state = PoseStamped()
        self.GNSS_state2 = PoseStamped()
        self.NDT_state = PoseStamped()
        self.Est_trajectory = Path()    
        self.GNSS_trajectory = Path()
        self.ndt_trajectory = Path()
        self.tf_Buffer = tf2_ros.Buffer()
        self.listner = tf2_ros.TransformListener(self.tf_Buffer)
        self.init_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
                
    def tf_pose(self,obj, frame_id, child_id= "base_link"):
        br2 = tf2_ros.TransformBroadcaster()
        t= TransformStamped()
        t.header.stamp= obj.header.stamp
        t.header.frame_id = frame_id
        t.child_frame_id = child_id
        t.transform.translation.x = obj.pose.position.x
        t.transform.translation.y = obj.pose.position.y
        t.transform.translation.z = obj.pose.position.z
        
        t.transform.rotation.x = obj.pose.orientation.x
        t.transform.rotation.y = obj.pose.orientation.y
        t.transform.rotation.z = obj.pose.orientation.z
        t.transform.rotation.w = obj.pose.orientation.w
        
        br2.sendTransform(t)
        
         
        
    def pose_generation(self,obj, state, frame_id, ori = True):
        obj.header.frame_id = frame_id
        obj.header.stamp = rospy.Time.now()
        obj.pose.position.x = state[0]
        obj.pose.position.y = state[1]
        obj.pose.position.z = 0
        if ori == True:
            q = tf_conversions.transformations.quaternion_from_euler(0.0,0.0,state[3]*m.pi/180)
            obj.pose.orientation.x = q[0]
            obj.pose.orientation.y = q[1]
            obj.pose.orientation.z = q[2]
            obj.pose.orientation.w = q[3]
            self.tf_pose(obj=obj, frame_id= frame_id, child_id="base_link")
        else:
            q = tf_conversions.transformations.quaternion_from_euler(0.0,0.0,0.0)
            obj.pose.orientation.x = q[0]
            obj.pose.orientation.y = q[1]
            obj.pose.orientation.z = q[2]
            obj.pose.orientation.w = q[3]
            self.tf_pose(obj=obj, frame_id= frame_id, child_id="base_link_GPS")
            
        return obj
    
    
    # def pose_listner(self, obj):
    #     tf_buffer = tf2_ros.Buffer()
    #     tf_listner = tf2_ros.TransformListener(tf_buffer)
        
    #     obj.header.frame_id = 
    
    def trajectory_generation_imu(self, state):
        self.Est_trajectory.header = state.header
        self.Est_trajectory.poses.append(state)
        self.ekf_path_pub.publish(self.Est_trajectory)
        
        
    def trajectory_generation_gnss(self, state):
        self.GNSS_trajectory.header = state.header
        self.GNSS_trajectory.poses.append(state)
        self.gnss_path_pub.publish(self.GNSS_trajectory)
    
    def trajectory_generation_ndt(self, state):
        self.ndt_trajectory.header.frame_id = "local"
        self.ndt_trajectory.header.stamp = rospy.Time.now()
        self.ndt_trajectory.poses.append(state)
        self.ndt_path_pub.publish(self.ndt_trajectory)
    
        
    def Est_publish(self,state, update):
        self.Est_state = self.pose_generation(obj=PoseStamped(), state=state, frame_id = "local", ori = True)
        self.ekf_pub.publish(self.Est_state)
        try:
            trans = self.tf_Buffer.transform(self.Est_state,"map",timeout=rospy.Duration(5.0))
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        self.state_pub.publish(trans)
        self.current_pose = trans
        self.trajectory_generation_imu(state = self.Est_state)
        
        if update:
            trans2 = PoseWithCovarianceStamped()
            trans2.header = self.GNSS_state.header
            trans2.pose.pose.position = self.GNSS_state.pose.position
            trans2.pose.pose.orientation = trans.pose.orientation
            
            self.init_pub.publish(trans2)
       
    def GNSS_publish(self,state, update, orientation):
        self.GNSS_state = self.pose_generation(obj=PoseStamped(), state=state, frame_id = "local", ori = False)
        self.gps_pub.publish(self.GNSS_state)
        self.trajectory_generation_gnss(state = self.GNSS_state)
        # add
        
        if update:
            
            self.GNSS_state2.header.frame_id = "local"
            self.GNSS_state2.header.stamp = rospy.Time.now()
            self.GNSS_state2.pose.position.x = state[0]
            self.GNSS_state2.pose.position.y = state[1]
            self.GNSS_state2.pose.position.z = 0
            q = tf_conversions.transformations.quaternion_from_euler(0.0,0.0,orientation*m.pi/180)
            self.GNSS_state2.pose.orientation.x = q[0]
            self.GNSS_state2.pose.orientation.y = q[1]
            self.GNSS_state2.pose.orientation.z = q[2]
            self.GNSS_state2.pose.orientation.w = q[3]
            
            try:
                trans = self.tf_Buffer.transform(self.GNSS_state2,"map",timeout=rospy.Duration(5.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            
            trans2 = PoseWithCovarianceStamped()
            trans2.header = trans.header
            trans2.pose.pose = trans.pose
            
            self.init_pub.publish(trans2)       
        
    def NDT_publish(self,state):
        self.trajectory_generation_ndt(state = state)
        

        