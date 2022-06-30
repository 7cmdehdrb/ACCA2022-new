from time import sleep
import numpy as np
import random
from matplotlib import pyplot as plt


class Kalman(object):
    def __init__(self):

        self.__A = np.array([[1, 0], [0, 1]])   # model matrix
        self.__H = np.array([[1, 0], [0, 1]])
        self.__Q = np.array([[0, 0], [0, 1]])
        self.__R = np.array([[1, 0], [0, 1]])
        self.x = np.array([[0, 0], [0, 0]])  # initial value
        self.P = np.array([[6, 0], [0, 1]])

    def inv(self, matrix):
        return np.linalg.inv(matrix)

    def kalmanFilter(self, z):
        # z : sensor value

        xp = np.dot(self.__A, self.x)
        Pp = np.dot(np.dot(self.__A, self.P), self.__A.T) + self.__Q
        K = np.dot(np.dot(Pp, self.__H.T), self.inv(
            self.__H * Pp * self.__H.T + self.__R))
        self.x = xp + np.dot(K, (z - np.dot(self.__H, xp)))
        self.P = Pp - np.dot(np.dot(K, self.__H), Pp)

        return self.x[0][0]


class Sensor(object):
    def __init__(self):
        self.value = 10

    def noise(self):
        return random.random() * 10

    def sense(self):
        return self.value + self.noise()


if __name__ == "__main__":
    kalman = Kalman()
    sensor = Sensor()

    x = []
    data = []
    sensors = []

    for i in range(1000):
        z = sensor.sense()
        val = kalman.kalmanFilter(z)

        x.append(i)
        data.append(val)
        sensors.append(z)

    plt.plot(x, data)
    plt.scatter(x, sensors, c="r", s=1)

    plt.show()
