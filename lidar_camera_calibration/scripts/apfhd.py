import rospy
import yaml
from sensor_msgs.msg import CameraInfo

def yaml_to_CameraInfo():
    """
    Parse a yaml file containing camera calibration data (as produced by 
    rosrun camera_calibration cameracalibrator.py) into a 
    sensor_msgs/CameraInfo msg.
    
    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data
    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = 680
    camera_info_msg.height = 480
    camera_info_msg.K = calib_data["camera_matrix"][746.283639, 0, 629.870177,0, 745.989369, 407.997170,0, 0, 1]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

if __name__ == "__main__":
    # Get fname from command line (cmd line input required)

    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo()

    # Initialize publisher node
    rospy.init_node("camera_info_publisher", anonymous=True)
    publisher = rospy.Publisher("camera_info", CameraInfo, queue_size=10)
    rate = rospy.Rate(10)

    # Run publisher
    while not rospy.is_shutdown():
        publisher.publish(camera_info_msg)
        rate.sleep()