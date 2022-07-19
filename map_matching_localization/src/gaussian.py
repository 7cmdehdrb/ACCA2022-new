#!/usr/bin/env python

import rospy
from matplotlib import pyplot as plt
from sensor_msgs.msg import NavSatFix
import math as m
import numpy as np


class Gaussian(object):
    def __init__(self, x, mean, sigma):
        self.x = x
        self.mean = mean
        self.sigma = sigma

        if x is not None:
            self.value = self.calculateGaussian()

    def calculateGaussian(self):
        return (1 / np.sqrt(2. * m.pi * self.sigma ** 2.)) * \
            np.exp(-(self.x - self.mean) ** 2. / (2. * self.sigma ** 2))


def gaussianConvolution(g1, g2):
    mean = g1.mean + (g1.sigma ** 2 * (g2.mean - g1.mean)) / \
        (g1.sigma ** 2 + g2.sigma ** 2)
    if g1.sigma == g2.sigma:
        sigma = g1.sigma
    else:
        sigma = g1.sigma ** 2 - (g1.sigma ** 4) / \
            (g1.sigma ** 2 - g2.sigma ** 2)
    return Gaussian(g1.x, mean, abs(sigma))


def measure(lat1, lon1, lat2, lon2):
    R = 6378.137
    dLat = lat2 * m.pi / 180 - lat1 * m.pi / 180
    dLon = lon2 * m.pi / 180 - lon1 * m.pi / 180
    a = m.sin(dLat/2) * m.sin(dLat/2) + \
        m.cos(lat1 * m.pi / 180) * m.cos(lat2 * m.pi / 180) * \
        m.sin(dLon/2) * m.sin(dLon/2)
    c = 2 * m.atan2(m.sqrt(a), m.sqrt(1-a))
    d = R * c
    return d * 1000


if __name__ == "__main__":
    rospy.init_node("gaussian")

    x1 = np.arange(10 - 5, 10 + 5, 0.1)
    g1 = Gaussian(x1, 10, 0.5)
    g2 = Gaussian(x1, 8, float("inf"))
    g3 = gaussianConvolution(g1, g2)

    plt.figure()
    plt.plot(x1, g1.value, label="N(%.5f, %.5f)" %
             (g1.mean, g1.sigma), c="red")

    plt.plot(x1, g2.value, label="N(%.5f, %.5f)" %
             (g2.mean, g2.sigma), c="blue")

    plt.plot(x1, g3.value, label="N(%.5f, %.5f)" %
             (g3.mean, g3.sigma), c="green")
    plt.legend()

    plt.show()
