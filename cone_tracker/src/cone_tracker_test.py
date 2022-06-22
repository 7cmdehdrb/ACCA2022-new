#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import math as m
from matplotlib import pyplot as plt
from scipy.spatial import Delaunay


points = np.array([[0., 0.], [0., 1.1], [1., 0.], [1., 1.]])
tri = Delaunay(points)

indices = tri.simplices
vertices = points[indices]

# print(vertices)
# print(tri.simplices.copy())

for vertice in vertices:
    xs = []
    ys = []
    for i, pose in enumerate(vertice):
        x, y = pose
        xs.append(x)
        ys.append(y)
    xs.append(xs[0])
    ys.append(ys[0])
    plt.plot(xs, ys)


# plt.plot(points[:, 0], points[:, 1], "o")
plt.show()
