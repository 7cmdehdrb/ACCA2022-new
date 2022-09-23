#!/usr/bin/env python


from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import *
quat1 = Quaternion()
quat2 = Quaternion()

quat1.x, quat1.y, quat1.z, quat1.z = 0.0, 0.0, -0.427897806854, 0.903827122236

quat2.x, quat2.y, quat2.z, quat2.z = 0.0, 0.0, -0.498959320036, 0.866625407514

yaw1 = euler_from_quaternion([quat1.x, quat1.y, quat1.z, quat1.z])
yaw2 = euler_from_quaternion([quat2.x, quat2.y, quat2.z, quat2.z])

print('yaw1 : %s, yaw2: %s' % (yaw1, yaw2))
