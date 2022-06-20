from time import sleep
import rospy
from path_plan.msg import PathRequest

rospy.init_node('test_pub')


test_pub = rospy.Publisher("PathPoint", PathRequest, queue_size=1)
# r = rospy.Rate(10)
# i = 0
sleep(2)
# while not rospy.is_shutdown():
# i = i + 1
path = PathRequest()
path.start = 'A1'
path.end = 'B1'
path.path_id = 'A1C1'

test_pub.publish(path)
sleep(1)
# rospy.spin()
# print(path)


# print(i)

# r.sleep()
