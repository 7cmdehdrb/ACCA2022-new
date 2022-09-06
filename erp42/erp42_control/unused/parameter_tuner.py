#!/usr/bin/env python

from cProfile import label
import rospy
from matplotlib import pyplot as plt
import numpy as np


show_plot = False


vs = [5., 10., 15., 20.]
cs = [1.85, 1.45, 0.7, 0.4]
hs = [1.0, 1.0, 0.9, 0.75]


def function(a, b, c, x):
    return a * x ** 2 + b * x + c


def getParamFunction(vs, params):
    return np.polyfit(vs, params, 2)


class ParameterTuner(object):
    def __init__(self):
        self.__ca, self.__cb, self.__cc = getParamFunction(vs=vs, params=cs)
        self.__ha, self.__hb, self.__hc = getParamFunction(vs=vs, params=hs)

    def getCGain(self, vel):
        return function(self.__ca, self.__cb, self.__cc, vel)

    def getHDRGain(self, vel):
        return function(self.__ha, self.__hb, self.__hc, vel)

    def getGain(self, vel):
        return self.getCGain(vel), self.getHDRGain(vel)


x = np.arange(0., 25., 0.1)

a, b, c = np.polyfit(vs, cs, 2)
print(a, b, c)

plt.figure()
plt.scatter(vs, cs, c="r")
plt.plot(x, function(a, b, c, x), c="b")
plt.xlabel("Velocity")
plt.ylabel("C_Gain")

a, b, c = np.polyfit(vs, hs, 2)
print(a, b, c)

plt.figure()
plt.scatter(vs, hs, c="r")
plt.plot(x, function(a, b, c, x), c="b")
plt.xlabel("Velocity")
plt.ylabel("HDR_Gain")


plt.show()
