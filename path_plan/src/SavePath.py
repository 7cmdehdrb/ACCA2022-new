#!/usr/bin/env python

from tf.transformations import euler_from_quaternion
import rospy
from DB import *
from std_msgs.msg import Empty, String, UInt8
from path_plan.msg import PathRequest, PathResponse
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Path
from LoadPath import LoadPath

db_name = rospy.get_param("/SavePath/db_name", "/path2.db")


class SavePath():
    def __init__(self):
        self.path = PoseArray()
        self.Request = PathRequest()
        self.trig = False

    def pathCallback(self, msg):
        self.path = msg

    def pathRequestCallback(self, msg):
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


if __name__ == "__main__":
    rospy.init_node("SavePath")

    db = DB(db_name)
    save_path = SavePath()
    load_path = LoadPath(db)

    rospy.Subscriber("/path_request", PathRequest,
                     callback=save_path.pathRequestCallback)
    rospy.Subscriber("/create_global_path",
                     PoseArray, callback=save_path.pathCallback)

    check_pub = rospy.Publisher("/overwrite_check", String, queue_size=1)
    path_pub = rospy.Publisher("/reset_path", Empty, queue_size=1)
    existed_path_pub = rospy.Publisher("/existed_path", Path, queue_size=1)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if save_path.trig is True:
            # check path in DB
            # return count where path id :
            flag = db.check_path_id(save_path.Request.path_id)
            # save path
            if flag == 1:
                # already existed
                # check existed path
                load_path.Request = save_path.Request
                load_path.bringPath()
                load_path.toRosPath()
                existed_path_pub.publish(load_path.path)

                check_pub.publish(
                    'Path is already existed\nDo you want to overwrite data?')
                ans = rospy.wait_for_message("/overwrite_ans", UInt8)  # ?
                if ans.data == 1:
                    # YES
                    try:
                        db.deletePath(save_path.Request.path_id)
                        path = save_path.poseArrayToPath(
                            poses=save_path.path)
                        db.savePath(path)

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
