#!/usr/bin/env python2.7
# -*- coding:utf-8 -*-

# Python 2/3 compatibility
from __future__ import print_function

import rospy
# import tf2_ros

from tf import transformations as t

(trans, rot) = transformer.lookupTransform(frame1, frame2, rospy.Time(0))
transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
inversed_transform = t.inverse_matrix(transform)