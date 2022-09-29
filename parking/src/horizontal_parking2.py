#!/usr/bin/env python

from time import sleep
import rospy
import rospkg
import numpy as np
import math as m
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from parking_area_selector import ParkingAreaSelector
from parking_area import ParkingArea
from cubic_spline_planner import calc_spline_course
from reeds_shepp_path_planning import reeds_shepp_path_planning
from enum import Enum

# msgs
from std_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from visualization_msgs.msg import *
from path_plan.msg import *
from erp42_control.msg import *
from load_parking_area import loadCSV

try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from state import State, OdomState
    from stanley import Stanley
except Exception as ex:
    rospy.logfatal(ex)


class ParkingState(Enum):
    NONE = 0
    SEARCH = 1
    ALIGN1 = 2
    ALIGN2 = 3
    PARKING1 = 4
    PARKING2 = 5
    PARKING3 = 6

    def __int__(self):
        return self.value


class HorizontalParking(object):
    def __init__(self, state, stanley, cmd_pub):
        self.state = state
        self.stanely = stanley

        file = rospkg.RosPack().get_path("parking") + "/parking/" + \
            rospy.get_param("/horizontal_parking/file", "hor_parking.csv")
        self.parking_areas = loadCSV(file)
        self.parking_idx = 0

        self.parking_state = ParkingState.ALIGN1
        self.target_idx = 0

        self.alignPath1, self.alignPath2 = self.createAlignPath()

        # ETC
        self.cmd_pub = cmd_pub
        self.cmd_msg = ControlMessage()
        self.parking_pub = rospy.Publisher(
            "/horizontal_parking/parking_area", MarkerArray, queue_size=5
        )
        self.path_pub = rospy.Publisher(
            "/horizontal_parking/path", Path, queue_size=1
        )
        self.test_pub = rospy.Publisher(
            "/horizontal_parking/test", PointStamped, queue_size=1
        )

        for i in range(10):
            self.publishParkingArea()
            sleep(0.1)

    # Loop : Stop for <duration> secs
    def wait_for_stop(self, duration, brake=50):
        current_time = rospy.Time.now()
        last_time = rospy.Time.now()

        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            dt = (current_time - last_time).to_sec()

            if dt > duration:
                last_time = current_time
                return 0

            self.cmd_msg = ControlMessage(0, 0, 2, 0, 0, brake, 0)
            self.cmd_pub.publish(self.cmd_msg)

            r.sleep()

    def control(self):
        msg = ControlMessage()

        reverse = False

        if self.parking_state == ParkingState.NONE:
            return 0

        elif self.parking_state == ParkingState.SEARCH:
            pass

        elif self.parking_state == ParkingState.ALIGN1:
            msg.Speed = int(5)

            di, self.target_idx = self.stanely.stanley_control(self.state, self.alignPath1.cx,
                                                               self.alignPath1.cy, self.alignPath1.cyaw, self.target_idx, reverse=reverse)
            self.publishPath(
                self.alignPath1.cx, self.alignPath1.cy, self.alignPath1.cyaw
            )

            if self.target_idx >= len(self.alignPath1.cx) - 1:
                self.target_idx = 0
                self.wait_for_stop(duration=3, brake=50)
                self.parking_state = ParkingState.ALIGN2

        elif self.parking_state == ParkingState.ALIGN2:
            reverse = True
            msg.Speed = int(3)

            di, self.target_idx = self.stanely.stanley_control(self.state, self.alignPath2.cx,
                                                               self.alignPath2.cy, self.alignPath2.cyaw, self.target_idx, reverse=reverse)

            self.publishPath(
                self.alignPath2.cx, self.alignPath2.cy, self.alignPath2.cyaw
            )

            if self.target_idx >= len(self.alignPath2.cx) - 1:
                self.target_idx = 0
                self.wait_for_stop(duration=3, brake=50)
                self.parking_state = ParkingState.PARKING1

        elif self.parking_state == ParkingState.PARKING1:
            reverse = True
            msg.Speed = int(3)

            di = 30

            target_area = self.parking_areas[self.parking_idx]
            _, _, parking_yaw = euler_from_quaternion(
                [target_area.orientation.x, target_area.orientation.y, target_area.orientation.z, target_area.orientation.w])

            heading_vec = np.array(
                [m.cos(self.state.yaw), m.sin(self.state.yaw)])
            parking_vec = np.array([m.cos(parking_yaw), m.sin(parking_yaw)])

            theta = abs(m.acos(np.dot(heading_vec, parking_vec)))

            if m.radians(45) - theta < m.radians(5):
                self.wait_for_stop(5)
                self.parking_state = ParkingState.PARKING2

        elif self.parking_state == ParkingState.PARKING2:
            reverse = True
            msg.Speed = int(3)

            di = 0

            target_area = self.parking_areas[self.parking_idx]
            _, _, parking_yaw = euler_from_quaternion(
                [target_area.orientation.x, target_area.orientation.y, target_area.orientation.z, target_area.orientation.w])

            area_reference_point = Point(
                target_area.position.x +
                (target_area.scale.x / 2.0) * m.cos(parking_yaw),
                target_area.position.y +
                (target_area.scale.x / 2.0) * m.sin(parking_yaw),
                0.0
            )

            self.publishPoint(area_reference_point)

            car_reference_point = Point(
                self.state.x + 2.3 * m.cos(self.state.yaw),
                self.state.y + 2.3 * m.sin(self.state.yaw),
                0.0
            )

            car_vec = np.array(
                [car_reference_point.x - area_reference_point.x,
                    car_reference_point.y - area_reference_point.y]
            )
            parking_vec = np.array([m.cos(parking_yaw), m.sin(parking_yaw)])

            dot = np.dot(car_vec, parking_vec)
            print(dot)

            if dot < 0:
                self.wait_for_stop(5)
                self.parking_state = ParkingState.PARKING3

        elif self.parking_state == ParkingState.PARKING3:
            reverse = True
            msg.Speed = int(3)

            di = -30

            target_area = self.parking_areas[self.parking_idx]
            _, _, parking_yaw = euler_from_quaternion(
                [target_area.orientation.x, target_area.orientation.y, target_area.orientation.z, target_area.orientation.w])

            heading_vec = np.array(
                [m.cos(self.state.yaw), m.sin(self.state.yaw)])
            parking_vec = np.array([m.cos(parking_yaw), m.sin(parking_yaw)])

            theta = abs(m.acos(np.dot(heading_vec, parking_vec)))

            if theta < m.radians(5):
                self.wait_for_stop(5)
                self.parking_state = ParkingState.NONE

        di = np.clip(di, -m.radians(30), m.radians(30))
        msg.Steer = int(m.degrees(di * (-1.0 if reverse is False else 1.0)))
        msg.Gear = 2 if reverse is False else 1

        self.cmd_pub.publish(msg)

    def publishParkingArea(self):
        msg = MarkerArray()

        for i, parking in enumerate(self.parking_areas):
            msg.markers.append(parking.parseMarker(id=i, duration=int(1)))

        self.parking_pub.publish(msg)

    def publishPoint(self, point):
        msg = PointStamped(Header(None, rospy.Time.now(), "map"), point)
        self.test_pub.publish(msg)

    def publishPath(self, cx, cy, cyaw):
        path = Path()

        header = Header(None, rospy.Time.now(), "map")

        path.header = header
        for i in range(len(cx)):
            quat = quaternion_from_euler(0.0, 0.0, cyaw[i])

            p = PoseStamped()
            p.header = header
            p.pose.position = Point(cx[i], cy[i], 0.0)
            p.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

            path.poses.append(p)

        self.path_pub.publish(path)

    def createAlignPath(self):
        target_area = self.parking_areas[self.parking_idx]

        start = Point(self.state.x, self.state.y, 0.0)

        _, _, parking_yaw = euler_from_quaternion(
            [target_area.orientation.x, target_area.orientation.y, target_area.orientation.z, target_area.orientation.w])

        left_side_point = Point(
            target_area.position.x +
            ((target_area.scale.y / 2.0) + 1.0) *
            m.cos(parking_yaw + m.pi / 2.0),
            target_area.position.y +
            ((target_area.scale.y / 2.0) + 1.0) *
            m.sin(parking_yaw + m.pi / 2.0),
            0.0
        )

        left_top_point = Point(
            left_side_point.x + (target_area.scale.x /
                                 2.0) * m.cos(parking_yaw),
            left_side_point.y + (target_area.scale.x /
                                 2.0) * m.sin(parking_yaw),
            0.0
        )

        margin_point = Point(
            left_top_point.x + (3.0) * m.cos(parking_yaw),
            left_top_point.y + (3.0) * m.sin(parking_yaw),
            0.0
        )

        cx, cy, cyaw, _, _ = calc_spline_course(
            [start.x, left_side_point.x, left_top_point.x, margin_point.x],
            [start.y, left_side_point.y, left_top_point.y, margin_point.y],
            0.1
        )

        align_path1 = PathResponse(None, None, None, cx, cy, cyaw)

        cx, cy, cyaw, _, _ = calc_spline_course(
            [margin_point.x, left_top_point.x],
            [margin_point.y, left_top_point.y],
            0.1
        )

        align_path2 = PathResponse(None, None, None, cx, cy, cyaw)

        return align_path1, align_path2


if __name__ == "__main__":
    rospy.init_node("horizontal_parking")

    state = State(odometry_topic="/ndt_matching/ndt_pose", hz=30, test=False)
    stanley = Stanley()

    cmd_pub = rospy.Publisher(
        "/cmd_msg", ControlMessage, queue_size=1)

    hp = HorizontalParking(state, stanley, cmd_pub)

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        hp.control()
        r.sleep()
