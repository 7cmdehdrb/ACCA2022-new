import os
from re import search
import sys
import rospy
import rospkg
import numpy as np
import math as m
from geometry_msgs.msg import *
from path_plan.msg import PathResponse
from erp42_control.msg import ControlMessage
from cubic_spline_planner import calc_spline_course
from abc import *
from enum import Enum

try:
    sys.path.append(rospkg.RosPack().get_path("erp42_control") + "/src")
    from stanley import Stanley
    from state import State, OdomState
except Exception as ex:
    rospy.logfatal("Import Error : Vertical Parking")
    rospy.logfatal(ex)


class ParkingState(Enum):
    Searching = 0   # find empty parking lot
    Reset = 1       # go back to start point & path plan
    Parking = 2     # tracking parking path
    Backward = 3    # end
    End = 4


class VerticalParkingBase(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__(self, state=OdomState(), stanley=Stanley()):
        self.state = state
        self.stanley = stanley
        self.path = PathResponse()
        self.local_path = PathResponse()

        self.startPoint = None

        self.parking_state = ParkingState.Searching

        self.target_idx = 0

    @abstractmethod
    def createPath(self, end_point=Point()):
        cx, cy, cyaw, _, _ = calc_spline_course([self.state.x, end_point.x], [
            self.state.y, end_point.y], ds=0.1)

        path = PathResponse()
        path.cx = cx
        path.cy = cy
        path.cyaw = cyaw

        return PathResponse()

    @abstractmethod
    def makeControlMessage(self, path):
        di, target_idx = self.stanley.stanley_control(
            state=self.state,
            cx=path.cx,
            cy=path.cy,
            cyaw=path.cyaw,
            last_target_idx=self.target_idx
        )

        self.target_idx = target_idx

        di = np.clip(di, -m.radians(30.0), m.radians(30.0))

        is_end = target_idx > len(self.path.cx) * 0.95

        return ControlMessage(0, 0, 2, 0, di, 0, 0), is_end

    @abstractmethod
    def main(self):
        cmd = ControlMessage()

        if self.parking_state.Searching:

            if self.startPoint is None:
                self.startPoint = Point(self.state.x, self.state.y, 0.)

            self.path = self.createPath(Point(0., 0., 0.))
            cmd, is_end = self.makeControlMessage(self.path)

            if is_end:
                self.parking_state = ParkingState.Reset

        elif self.parking_state.Reset:
            self.path = self.createPath(self.startPoint)
            cmd, is_end = self.makeControlMessage(self.path)
            # Call Function for Local Path Planning

            if is_end:
                self.parking_state = ParkingState.Parking

        elif self.parking_state.Parking:
            cmd, is_end = self.makeControlMessage(self.local_path)

            if is_end:
                self.startPoint = Point(self.state.x, self.state.y, 0.0)
                self.parking_state = ParkingState.Backward

        elif self.parking_state.Backward:
            distance = np.hypot(
                self.state.x - self.startPoint.x,
                self.state.y - self.startPoint.y
            )

            if distance > 10:
                self.parking_state = ParkingState.End
            else:
                cmd = ControlMessage(0, 0, 1, 5, 0, 0, 0)

        elif self.parking_state.End:
            pass

        else:
            rospy.logfatal("Invalid Parking State")

        return cmd


if __name__ == "__main__":
    rospy.init_node("test")

    state = OdomState("/odometry/kalman")
    stanley = Stanley()

    while True:
        pass
