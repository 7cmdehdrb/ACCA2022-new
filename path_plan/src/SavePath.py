#!/usr/bin/env python

from ast import Load
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
from DB import *
from std_msgs.msg import Empty, String, UInt8
from path_plan.msg import PathRequest, PathResponse
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Path
from LoadPath import LoadPath


class SavePath():
    def __init__(self):
        self.path = PoseArray()
        self.Request = PathRequest()
        self.trig = False

    def pathCallback(self, msg):
        self.path = msg

    def PathPointCallback(self, msg):
        self.Request = msg
        self.trig = True

    def poseArrayToPath(self, poses):
        path = PathResponse()
        # poses = PoseArray()

        path.start = self.Request.start
        path.end = self.Request.end
        path.path_id = self.Request.path_id

        if len(poses.poses) < 100:
            raise Exception("path is too short")

        for pose in poses.poses:

            px = pose.position.x
            py = pose.position.y

            ox = pose.orientation.x
            oy = pose.orientation.y
            oz = pose.orientation.z
            ow = pose.orientation.w

            _, _, yaw = euler_from_quaternion(
                [ox, oy, oz, ow])

            path.cx.append(px)
            path.cy.append(py)
            path.cyaw.append(yaw)

        rospy.logwarn(len(path.cx))
        return path

    def bringPath(self):
        # bring path
        self.path_id = db.bring_path_id(
            self.Request.start, self.Request.end)
        self.path_info = db.bring_pathinfo(self.path_id)
        # [[x, y, yaw]]

    def toRosPath(self):
        # ros path publish
        self.path = Path()
        self.path.header.frame_id = "map"
        self.path.header.stamp = rospy.Time.now()

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

            self.path.poses.append(pose)


if __name__ == "__main__":
    rospy.init_node("SavePath")

    db = DB()
    save_path = SavePath()
    # load_path = LoadPath(db)

    rospy.Subscriber("/PathPoint", PathRequest,
                     callback=save_path.PathPointCallback)
    rospy.Subscriber("/create_global_path",
                     PoseArray, callback=save_path.pathCallback)

    check_pub = rospy.Publisher("/saving_check", String, queue_size=1)
    path_pub = rospy.Publisher("/reset_path", Empty, queue_size=1)
    existing_path_pub = rospy.Publisher("/existing_path", Path, queue_size=1)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if save_path.trig is True:
            # check path in DB
            # return count where path id :
            flag = db.check_path_id(save_path.Request.path_id)
            # save path
            if flag == 1:
                # already existed
                # existing path pub
                save_path.bringPath()
                save_path.toRosPath()
                existing_path_pub.publish(save_path.path)

                check_pub.publish(
                    'Path is already existed\nDo you want to overwrite data?')
                ans = rospy.wait_for_message("/saving_ans", UInt8)  # ?
                if ans.data == 1:
                    # YES
                    try:
                        db.deletePath(save_path.Request.path_id)
                        path = save_path.poseArrayToPath(
                            poses=save_path.path)
                    except Exception as ex:
                        rospy.logwarn(ex)
            else:
                # not exist
                # TO DO : Save path
                path = save_path.poseArrayToPath(poses=save_path.path)
                db.savePath(path)

            rospy.loginfo("SAVE INTO DB!")
            path_pub.publish()

            save_path.trig = False

        r.sleep()
