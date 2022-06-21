#!/usr/bin/env python

import rospy
import numpy as np
import math as m
from geometry_msgs.msg import Point
from path_plan.msg import PathRequest, PathResponse
from state import State


class PathSelector(object):
    def __init__(self, state=State()):
        self.state = state
