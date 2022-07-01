#!/usr/bin/env python

import rospy
from matplotlib import pyplot as plt
import math as m
import numpy as np


class Gaussian(object):
    def __init__(self, x, mean, sigma):
        self.x = x
        self.mean = mean
        self.sigma = sigma

        # self.value = self.calculateGaussian()

    def calculateGaussian(self):
        return (1 / np.sqrt(2. * m.pi * self.sigma ** 2.)) * \
            np.exp(-(x - self.mean) ** 2. / (2. * self.sigma ** 2))


def gaussianConvolution(g1, g2):
    mean = g1.mean + (g1.sigma ** 2 * (g2.mean - g1.mean)) / \
        (g1.sigma ** 2 + g2.sigma ** 2)
    if g1.sigma == g2.sigma:
        sigma = g1.sigma
    else:
        sigma = g1.sigma ** 2 - (g1.sigma ** 4) / \
            (g1.sigma ** 2 - g2.sigma ** 2)
    return Gaussian(g1.x, mean, sigma)


if __name__ == "__main__":
    rospy.init_node("gaussian")

    x = np.arange(-15, 15, 0.01)

    legend = []

    g1 = Gaussian(x, 0, 1)
    plt.plot(x, g1.value)
    legend.append("N(%.2f, %.2f)" % (g1.mean, g1.sigma))

    g2 = Gaussian(x, 1, 10)
    plt.plot(x, g2.value)
    legend.append("N(%.2f, %.2f)" % (g2.mean, g2.sigma))

    plt.plot(x, Gaussian(x, 0, 1).value * Gaussian(x, 1, 3).value)
    legend.append("Convolution")

    conv = gaussianConvolution(g1, g2)
    plt.plot(x, conv.value)
    legend.append("N(%.4f, %.4f)" % (conv.mean, conv.sigma))

    plt.legend(legend)
    plt.show()
