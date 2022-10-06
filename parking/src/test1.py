import numpy as np
import math as m

box_yaw = 0.5

t = np.array([[m.cos(box_yaw), -m.sin(box_yaw), 0, 23],
                [m.sin(box_yaw), m.cos(box_yaw), 0, 56],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

p = np.array([[0],[1.35],[0],[1]])


detect_p = np.dot(t, p)
print(detect_p)
print(np.shape(detect_p))
print(detect_p[0])
print(detect_p[1])