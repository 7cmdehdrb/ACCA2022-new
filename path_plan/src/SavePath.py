#!/usr/bin/env python

import tf
import rospy
from DB import *
from std_msgs.msg import Empty
from path_plan.msg import PathRequest, PathResponse
from geometry_msgs.msg import PoseArray, Pose


class SavePath():
    def __init__(self):
        # PathPoint table
        self.start = rospy.get_param("/SavePath/start", "B2")
        self.end = rospy.get_param("/SavePath/end", "A2")

        self.path_id = rospy.get_param(
            "/save_path/path_id", (self.start + self.end))

        rospy.Subscriber("/create_global_path",
                         PoseArray, callback=self.pathCallback)

        self.path = PoseArray()
        self.trig = True

    def pathCallback(self, msg):
        self.path = msg

    def poseArrayToPath(self, poses):
        path = PathResponse()
        # poses = PoseArray()

        path.start = self.start
        path.end = self.end
        path.path_id = self.path_id

        for pose in poses.poses:
            px = pose.position.x
            py = pose.position.y

            ox = pose.orientation.x
            oy = pose.orientation.y
            oz = pose.orientation.z
            ow = pose.orientation.w

            _, _, yaw = tf.transformations.euler_from_quaternion(
                [ox, oy, oz, ow])

            path.cx.append(px)
            path.cy.append(py)
            path.cyaw.append(yaw)
        rospy.logwarn(len(path.cx))
        return path


if __name__ == "__main__":
    rospy.init_node("SavePath")

    db = DB()
    save_path = SavePath()

    # create_table
    db.maketable()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():

        if save_path.trig is True:
            rospy.wait_for_message("/save_DB", Empty)
            # TO DO : Save path
            path = save_path.poseArrayToPath(poses=save_path.path)
            db.savePath(path)

            rospy.loginfo("SAVE INTO DB!")

            save_path.trig = False

        r.sleep()

        # continue
        # s_path.save_DB(idx)
        # if s_path.trig:
        #     rospy.loginfo("saving")
        #     for iter in range(len(s_path.rename.poses)):
        #         s_path.handleDate(iter)
        #         # db.makepoint(s_path.PathRequest.start,
        #         #              s_path.PathRequest.end, s_path.PathRequest.path_id)
        #         db.makepathinfo('B1C1', iter+1,
        #                         s_path.rename.poses[iter].position.x, s_path.rename.poses[iter].position.y, s_path.yaw)
        #     s_path.trig = False
        #     idx = False
        # else:
        #     rospy.loginfo('finished')
        #     break
