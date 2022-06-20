import rospy
from time import sleep
from DB import *
from path_plan.msg import PathRequest, PathResponse
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped


class LoadPath():
    def __init__(self, db):
        self.sub = rospy.Subscriber("/PathPoint", PathRequest,
                                    callback=self.RequestCallback)
        self.db = db
        self.Request = PathRequest()
        self.Response = PathResponse()
        self.ros = Path()
        self.trig = False

    def RequestCallback(self, msg):
        self.Request = msg
        # rospy.loginfo('receive a msg')

    def bringPath(self):
        # bring path
        self.path_id = self.db.bring_path_id(
            self.Request.start, self.Request.end)
        self.path_info = self.db.bring_pathinfo(self.path_id)
        # [[x, y, yaw]]

    def listToPath(self):
        # PathResponse publish path
        self.Response.start = self.Request.start
        self.Response.end = self.Request.end
        self.Response.path_id = self.Request.path_id

        for info in self.path_info:
            self.Response.cx.append(info[0])
            self.Response.cy.append(info[1])
            self.Response.cyaw.append(info[2])

        path_pub.publish(self.Response)

    def toRosPath(self):
        # ros path publish
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        for info in self.path_info:

            pose = PoseStamped()

            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()

            pose.pose.position.x = info[0]
            pose.pose.position.y = info[1]

            quat = quaternion_from_euler(0., 0., info[2])

            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            path.poses.append(pose)

        ros_path_pub.publish(path)

    def trigger(self):
        rospy.wait_for_message("/PathPoint", PathRequest)
        self.trig = True


if __name__ == "__main__":
    rospy.init_node("LoadPath")

    db = DB()
    l_path = LoadPath(db)

    path_pub = rospy.Publisher("Loaded_Path", PathResponse, queue_size=1)
    ros_path_pub = rospy.Publisher("ros_Path", Path, queue_size=1)

    r = rospy.Rate(0.1)
    while not rospy.is_shutdown():

        l_path.trigger()
        rospy.loginfo('trigger is True')

        while l_path.trig is True:
            try:
                l_path.bringPath()
            except:
                rospy.loginfo('wrong info')
                break

            l_path.listToPath()

            if l_path.sub.get_num_connections() > 0:
                l_path.toRosPath()
                rospy.loginfo('ros path published')

            l_path.trig = False
            rospy.loginfo('trigger is False')
