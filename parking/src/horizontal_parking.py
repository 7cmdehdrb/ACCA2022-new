#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import math as m
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from parking_area_selector import ParkingAreaSelector
from cubic_spline_planner import calc_spline_course
from reeds_shepp_path_planning import reeds_shepp_path_planning
from enum import Enum

# msgs
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import *
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from path_plan.msg import PathResponse
from erp42_control.msg import ControlMessage
from load_parking_area import loadCSV

try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from state import State, OdomState
    from stanley import Stanley
except Exception as ex:
    rospy.logfatal(ex)


class HorizontalParkingState(Enum):
    Search = 0
    Back = 1
    Straight = 2
    Reverse = 3
    Home = 4
    Final = 5
    Out = 6
    End = 7
    Break = 8

    def __int__(self):
        return self.value


class HorizontalParking(object):
    def __init__(self, state, cmd_pub, stanley, search_path, file_path):
        self.state = state

        self.search_path = None
        self.back_path = None

        self.stanley = stanley
        self.target_idx = 0

        self.r = rospy.Rate(30)

        # For test
        # self.stanley.setCGain(1.0)
        # self.stanley.setHdrRatio(1.0)

        self.circles_pub = rospy.Publisher(
            "/horizontal_parking/circles", MarkerArray, queue_size=5
        )
        self.path_pub = rospy.Publisher(
            "/horizontal_parking/parking_path", Path, queue_size=5
        )
        self.parking_pub = rospy.Publisher(
            "/horizontal_parking/parking_area", MarkerArray, queue_size=5
        )

        self.horizontal_parking_state = HorizontalParkingState.Search
        self.parking_areas = loadCSV(file_path)

        self.cmd_pub = cmd_pub
        self.cmd_msg = ControlMessage()

        # TO DO : Put function for decide parking lot
        self.idx = -1
        self.parking_area_selector = ParkingAreaSelector(
            self.state, self.parking_areas)

    def setSearchPath(self, search_path):
        self.search_path = search_path  # PathResponse
        self.back_path = PathResponse(None, None, None, list(reversed(self.search_path.cx)), list(
            reversed(self.search_path.cy)), [self.search_path.cyaw[i] + m.pi for i in range(len(self.search_path.cyaw) - 1, -1, -1)])

        return True

    def publishParkingArea(self):
        msg = MarkerArray()

        for i, parking in enumerate(self.parking_areas):
            msg.markers.append(parking.parseMarker(id=i, duration=int(1)))

        self.parking_pub.publish(msg)

    # Loop : Stop for <duration> secs
    def wait_for_stop(self, duration, brake=70):
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

    def createPath(self, circle1, circle2, selected_parking_area):
        _, _, yaw = euler_from_quaternion([
            selected_parking_area.orientation.x,
            selected_parking_area.orientation.y,
            selected_parking_area.orientation.z,
            selected_parking_area.orientation.w
        ])

        # std_vec : Unit Vector, cross_vec : Vector from circle1 point to contact point
        std_vec = np.array([m.cos(yaw), m.sin(yaw)])
        cross_vec = np.array(
            [circle1.pose.position.x - circle2.pose.position.x, circle1.pose.position.y - circle2.pose.position.y])

        # orientation gap between heading angle of parking lot and contact point
        theta1 = m.acos(np.dot(std_vec, cross_vec) /
                        ((np.hypot(std_vec[0], std_vec[1])) *
                        np.hypot(cross_vec[0], cross_vec[1])))

        # theta range for circle 1
        yaw_start = yaw + m.pi / 2
        yaw_end = yaw + theta1
        yaw_range = np.arange(yaw_start, yaw_end, 0.01 *
                              (1.0 if yaw_end > yaw_start else -1.0))

        # Set start point : left side of green circlr
        start_x = circle2.pose.position.x + \
            circle2.scale.x / 2.0 * \
            m.cos(yaw_range[0]) + (circle2.scale.x / 2.0 + 0.0) * m.cos(yaw)
        start_y = circle2.pose.position.y + \
            circle2.scale.x / 2.0 * \
            m.sin(yaw_range[0]) + (circle2.scale.x / 2.0 + 0.0) * m.sin(yaw)

        # +n m of start point
        start_x2 = circle2.pose.position.x + \
            circle2.scale.x / 2.0 * \
            m.cos(yaw_range[0]) + (circle2.scale.x / 2.0 + 1.0) * m.cos(yaw)
        start_y2 = circle2.pose.position.y + \
            circle2.scale.x / 2.0 * \
            m.sin(yaw_range[0]) + (circle2.scale.x / 2.0 + 1.0) * m.sin(yaw)

        xs1 = [circle2.pose.position.x + circle2.scale.x /
               2.0 * m.cos(y) for y in yaw_range]
        ys1 = [circle2.pose.position.y + circle2.scale.x /
               2.0 * m.sin(y) for y in yaw_range]

        # change vector heading vector
        cross_vec = -cross_vec

        # orientation gap between heading angle of parking lot and contact point
        theta2 = m.acos(np.dot(std_vec, cross_vec) /
                        ((np.hypot(std_vec[0], std_vec[1])) *
                        np.hypot(cross_vec[0], cross_vec[1])))

        # theta range for circle 2
        yaw_start = yaw - theta2
        yaw_end = yaw - m.pi / 2
        yaw_range = np.arange(yaw_start, yaw_end, 0.01 *
                              (1.0 if yaw_end > yaw_start else -1.0))

        # Set end point : +n m(relative with car height) of end point of parking lot
        fs = float(rospy.get_param("/horizontal_parking/fs", 1.2))
        end_x = selected_parking_area.position.x + ((selected_parking_area.scale.x / 2.0) *
                                                    m.cos(yaw + m.pi)) - ((2.02 / 2.0) * m.cos(yaw + m.pi) * fs)
        end_y = selected_parking_area.position.y + ((selected_parking_area.scale.x / 2.0) *
                                                    m.sin(yaw + m.pi)) - ((2.02 / 2.0) * m.sin(yaw + m.pi) * fs)

        end_x2 = selected_parking_area.position.x + (selected_parking_area.scale.x / 2.0) * \
            m.cos(yaw) - (2.02 / 2.0) * m.cos(yaw) * fs
        end_y2 = selected_parking_area.position.y + (selected_parking_area.scale.x / 2.0) * \
            m.sin(yaw) - (2.02 / 2.0) * m.sin(yaw) * fs

        xs2 = [circle1.pose.position.x +
               circle1.scale.x / 2.0 * m.cos(y) for y in yaw_range]
        ys2 = [circle1.pose.position.y +
               circle1.scale.x / 2.0 * m.sin(y) for y in yaw_range]

        _, _, gyaw = euler_from_quaternion(
            [circle1.pose.orientation.x, circle1.pose.orientation.y, circle1.pose.orientation.z, circle1.pose.orientation.w])

        # Path : current state to horizontal position of lot
        # scx, scy, scyaw, _, _ = reeds_shepp_path_planning(
        #     sx=self.state.x,
        #     sy=self.state.y,
        #     syaw=self.state.yaw,
        #     gx=start_x,
        #     gy=start_y,
        #     gyaw=gyaw,
        #     maxc=0.5,
        #     step_size=0.1
        # )
        
        scx, scy, scyaw, _, _ = calc_spline_course(
            [self.state.x, start_x], [self.state.y, start_y], ds=0.1)

        # Path : Extra path for spath
        scx2, scy2, scyaw2, _, _ = calc_spline_course(
            [start_x, start_x2], [start_y, start_y2], ds=0.1)

        xs = [start_x2] + [start_x] + xs1 + xs2 + [end_x]
        ys = [start_y2] + [start_y] + ys1 + ys2 + [end_y]

        # Path : Reverse Path

        gcx, gcy, gcyaw, _, _ = calc_spline_course(xs, ys, 0.1)
        ocx, ocy, ocyaw, _, _ = calc_spline_course(
            list(reversed(xs)), list(reversed(ys)), 0.1)

        # Path : Homing Path
        hcx, hcy, hcyaw, _, _ = calc_spline_course(
            [end_x, end_x2], [end_y, end_y2], 0.1)

        # Path : Reverse Path for Final Position
        fcx, fcy, fcyaw, _, _ = calc_spline_course(
            [end_x2, end_x],
            [end_y2, end_y],
            0.1
        )

        straight_path = PathResponse(
            None, None, None, scx + scx2, scy + scy2, scyaw + scyaw2)
        reverse_path = PathResponse(None, None, None, gcx, gcy, gcyaw)
        home_path = PathResponse(None, None, None, hcx, hcy, hcyaw)
        final_path = PathResponse(None, None, None, fcx, fcy, fcyaw)
        out_path = PathResponse(None, None, None, ocx, ocy, ocyaw)

        cx = straight_path.cx + reverse_path.cx + \
            home_path.cx + final_path.cx + out_path.cx
        cy = straight_path.cy + reverse_path.cy + \
            home_path.cy + final_path.cy + out_path.cy
        cyaw = straight_path.cyaw + reverse_path.cyaw + \
            home_path.cyaw + final_path.cyaw + out_path.cyaw

        path = Path()
        path.header = Header(None, rospy.Time.now(), "map")

        for i in range(len(cx)):
            quat = quaternion_from_euler(0., 0., cyaw[i])

            p = PoseStamped()
            p.header = Header(None, rospy.Time.now(), "map")
            p.pose.position = Point(cx[i], cy[i], 0.)
            p.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

            path.poses.append(p)

        return straight_path, reverse_path, home_path, final_path, out_path, path

    # Make Two Circle from parking_area[idx]
    def getTwoCircle(self, idx):
        selected_parking_area = self.parking_areas[idx]

        h = selected_parking_area.scale.x   # 4.5m
        w = selected_parking_area.scale.y   # 1.7m
        reverse_threshold = np.clip(float(rospy.get_param(
            "/horizontal_parking/reverse_threshold", 0.15)), 0.0, 0.25)

        # ===== circle 1 =====
        circle1 = Marker()

        circle1.header = Header(None, rospy.Time.now(), "map")
        circle1.ns = "0"
        circle1.id = 0
        circle1.type = 3
        circle1.action = 0
        circle1.color = ColorRGBA(0, 0, 1, 0.3)

        center1 = selected_parking_area.position
        _, _, yaw = euler_from_quaternion([
            selected_parking_area.orientation.x,
            selected_parking_area.orientation.y,
            selected_parking_area.orientation.z,
            selected_parking_area.orientation.w
        ])

        r1 = w * 0.5

        cir_center1 = Point(
            center1.x - h * reverse_threshold * m.cos(yaw) +
            r1 * m.cos(yaw + m.pi / 2),
            center1.y - h * reverse_threshold * m.sin(yaw) +
            r1 * m.sin(yaw + m.pi / 2),
            0
        )

        circle1.pose.position = cir_center1
        circle1.pose.orientation = Quaternion(0, 0, 0, 1)
        circle1.scale = Vector3(r1 * 2.0, r1 * 2.0, 0.1)

        # ===== circle 2 =====
        circle2 = Marker()

        circle2.header = Header(None, rospy.Time.now(), "map")
        circle2.ns = "1"
        circle2.id = 1
        circle2.type = 3
        circle2.action = 0
        circle2.color = ColorRGBA(0, 1, 0, 0.3)

        # Calculate Contact Point (Circle1 - Straight Line)
        arrange = np.arange(0., h, 0.01)

        xs = center1.x + h * 0.5 * \
            m.cos(yaw) + r1 * 2 * m.cos(yaw + m.pi / 2) - \
            arrange * m.cos(yaw + m.pi / 2)
        ys = center1.y + h * 0.5 * \
            m.sin(yaw) + r1 * 2 * m.sin(yaw + m.pi / 2) - \
            arrange * m.sin(yaw + m.pi / 2)
        idx = np.argmin(
            np.abs(arrange - np.hypot(xs - center1.x, ys - center1.y)))

        cir_center2 = Point(
            xs[idx],
            ys[idx],
            0.0
        )

        r2 = np.hypot(circle1.pose.position.x -
                      cir_center2.x, circle1.pose.position.y - cir_center2.y) - r1

        circle2.pose.position = cir_center2
        circle2.pose.orientation = Quaternion(0, 0, 0, 1)
        circle2.scale = Vector3(r2 * 2.0, r2 * 2.0, 0.1)

        for i in range(30):
            msg = MarkerArray()
            msg.markers = [circle1, circle2]
            self.circles_pub.publish(msg)
            self.publishParkingArea()
            self.r.sleep()

        return circle1, circle2, selected_parking_area

    def loop(self):
        msg = ControlMessage()
        reverse = False
        
        print(self.horizontal_parking_state)

        if self.horizontal_parking_state == HorizontalParkingState.Search:
            msg.Speed = int(5)
            msg.Gear = int(2)
            self.current_path = self.search_path

            if self.parking_area_selector.flag is True:
                self.idx = self.parking_area_selector.target_idx
                self.target_idx = 0

                self.horizontal_parking_state = HorizontalParkingState.Back
                self.wait_for_stop(2, 70)

        elif self.horizontal_parking_state == HorizontalParkingState.Back:
            msg.Speed = int(5)
            msg.Gear = int(0)
            self.current_path = self.back_path

            reverse = True

            if self.idx == -1:
                # Fuck...
                rospy.logfatal("ERROR : FORCE PARKING - IDX 1")
                self.idx = 1

            if self.target_idx >= len(self.current_path.cx) - 30:
                circle1, circle2, selected_parking_area = self.getTwoCircle(
                    self.idx)
                self.straight_path, self.reverse_path, self.home_path, self.final_path, self.out_path, self.path = self.createPath(
                    circle1, circle2, selected_parking_area)

                self.current_path = self.straight_path

                for _ in range(30):
                    self.path_pub.publish(self.path)
                    self.r.sleep()

        elif self.horizontal_parking_state == HorizontalParkingState.Straight:
            msg.Speed = int(5)
            msg.Gear = int(2)
            self.current_path = self.straight_path

        elif self.horizontal_parking_state == HorizontalParkingState.Reverse:
            msg.Speed = int(3)
            msg.Gear = int(0)
            self.current_path = self.reverse_path

            reverse = True

        elif self.horizontal_parking_state == HorizontalParkingState.Home:
            msg.Speed = int(3)
            msg.Gear = int(2)
            self.current_path = self.home_path

        elif self.horizontal_parking_state == HorizontalParkingState.Final:
            msg.Speed = int(3)
            msg.Gear = int(0)
            self.current_path = self.final_path

            reverse = True

        elif self.horizontal_parking_state == HorizontalParkingState.Out:
            msg.Speed = int(3)
            msg.Gear = int(2)

            self.current_path = self.out_path

        elif self.horizontal_parking_state == HorizontalParkingState.End:
            self.wait_for_stop(3)

        elif self.horizontal_parking_state == HorizontalParkingState.Break:
            return 0

        else:
            rospy.logfatal("Invalid Horizontal Parking State")


        if self.target_idx >= len(self.current_path.cx) - 1:
            self.target_idx = len(self.current_path.cx) - 5

        try:

            di, self.target_idx = self.stanley.stanley_control(
                state=self.state,
                cx=self.current_path.cx,
                cy=self.current_path.cy,
                cyaw=self.current_path.cyaw,
                last_target_idx=self.target_idx,
                reverse=reverse
            )

            di = np.clip(di, -m.radians(30), m.radians(30))
            msg.Steer = m.degrees(di * (-1.0 if reverse is False else 1.0))

            car_vec = np.array([
                m.cos(self.state.yaw + m.pi if reverse is True else 0.0), m.sin(
                    self.state.yaw + m.pi if reverse is True else 0.0)
            ])
            position_vec = np.array(
                [self.current_path.cx[-1] - self.state.x,
                    self.current_path.cy[-1] - self.state.y]
            )

            theta = m.acos(np.dot(car_vec, position_vec) /
                        (np.hypot(car_vec[0], car_vec[1]) * np.hypot(position_vec[0], position_vec[1])))

            if self.target_idx >= len(self.current_path.cx) - 20:
                self.horizontal_parking_state = HorizontalParkingState(
                    int(self.horizontal_parking_state) + 1)
                self.target_idx = 0
                self.wait_for_stop(2)

        except Exception as ex:
            rospy.logwarn(ex)
            self.horizontal_parking_state = HorizontalParkingState(
                    int(self.horizontal_parking_state) + 1)
            self.target_idx = 0
            self.wait_for_stop(2)


        print("State : %d\tTarget idx : %d\tLength : %d" %
              (int(self.horizontal_parking_state), self.target_idx, len(self.current_path.cx)))

        self.cmd_msg = msg


if __name__ == "__main__":
    rospy.init_node("horizontal_parking")

    state = State(odometry_topic="/ndt_matching/ndt_pose", hz=30)
    stanley = Stanley()
    cmd_pub = rospy.Publisher(
        "/cmd_msg", ControlMessage, queue_size=1)

    # state, cmd_pub, stanley, search_path, file_path
    hp = HorizontalParking(state=state, cmd_pub=cmd_pub, stanley=stanley,
                           file_path="/home/acca/catkin_ws/src/ACCA2022-new/parking/parking/bs_parking.csv", search_path=None)

    hp.horizontal_parking_state = HorizontalParkingState.Straight
    hp.target_idx = 1
    hp.createPath()

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        hp.loop()
        cmd_pub.publish(hp.cmd_msg)
        r.sleep()
