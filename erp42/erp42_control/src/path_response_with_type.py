import rospy
import math as m
from path_plan.msg import PathRequest, PathResponse
from enum import Enum


class PathType(Enum):
    STRAIGHT = 0
    RIGHT = 1
    LEFT = 2
    NONE = 3


class PathResponseWithType(PathResponse):
    def __init__(self, *args, **kwds):
        super(PathResponseWithType, self).__init__(*args, **kwds)
        self.type = self.checkPathType()

    def checkPathType(self):
        if (self.start[0] == self.end[0]) is False:
            rospy.loginfo("Not Intersection!")
            return PathType.NONE
        else:
            # 0 : None, 1: -3.14 > + 3.14, 2: +3.14 > -3.14
            trig = 0
            average = 0.
            prev_average = 0.

            for i in range(len(self.cx) - 1):

                if trig == 0:
                    if self.cyaw[i] * self.cyaw[i + 1] < -9.:
                        # 1: -3.14 > + 3.14, 2: +3.14 > -3.14
                        if self.cyaw[i + 1] > 0.:
                            trig = 1
                        else:
                            trig = 2

                if trig == 1:
                    # -3.14 > +3.14
                    self.cyaw[i + 1] -= 2.0 * m.pi
                elif trig == 2:
                    # +3.14 > -3.14
                    self.cyaw[i + 1] += 2.0 * m.pi

                dyaw = self.cyaw[i + 1] - self.cyaw[i]

                alpha = (i) / (i + 1 + 0.0)
                average = alpha * prev_average + (1 - alpha) * dyaw
                prev_average = average

            if abs(average) < 0.003:
                rospy.loginfo("Guess Straight")
                return PathType.STRAIGHT
            elif average < 0.:
                rospy.loginfo("Guess Right")
                return PathType.RIGHT
            else:
                rospy.loginfo("Guess Left")
                return PathType.LEFT
