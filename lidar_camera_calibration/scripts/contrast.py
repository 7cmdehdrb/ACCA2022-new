from visualization_msgs.msg import MarkerArray, Marker

if __name__ == '__main__':
    rospy.init_node('contrast',anonymous=True)
    rospy.Subscriber("/adaptive_clustring/markers",MarkerArray,markersCallback)