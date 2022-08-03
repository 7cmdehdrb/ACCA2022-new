#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

def talker():
    pub = rospy.Publisher('bounding_boxes', BoundingBoxes, queue_size=10)
    rospy.init_node('yolo_falsedata', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        boxes = BoundingBoxes()
        boxes.header.stamp = rospy.Time.now()
        boxes.header.frame_id = 'world'
        boxes.image_header.stamp = rospy.Time.now()
        boxes.image_header.frame_id = 'world'
        box = BoundingBox()
        box.probability = 0.8
        box.xmin = 100
        box.xmax = 300
        box.ymin = 100
        box.ymax = 200
        box.id = 1
        boxes.bounding_boxes.append(box)
        pub.publish(boxes)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass