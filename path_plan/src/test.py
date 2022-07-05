#!/usr/bin/env python

import numpy as np

H = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
p = np.array([[5, 0, 0, 0], [0, 5, 0, 0], [0, 0, 5, 0], [0, 0, 0, 5]])

# print(np.dot(H, p))
A = np.dot(H, p)

print(np.dot(p, H.T))
