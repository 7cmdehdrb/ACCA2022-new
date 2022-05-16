#!/usr/bin/env python


from time import sleep
import rospy
import numpy as np
from math import ceil, floor
from sensor_msgs.msg import PointCloud2
from header import PointCloud
from point_cloud import array_to_xyz_pointcloud2f


class Grid(object):
    def __init__(self, xrange, yrange):

        self.xrange = xrange
        self.yrange = yrange

        self.__cx = None
        self.__cy = None
        self.__data = []

    @property
    def cx(self):
        return self.__cx

    @cx.setter
    def cx(self, value):
        self.__cx = value

    @property
    def cy(self):
        return self.__cy

    @cy.setter
    def cy(self, value):
        self.__cy = value

    @property
    def data(self):
        return self.__data

    def dataAppender(self, value):
        self.__data.append(value)

    # @data.setter
    # def data(self, value):
    #     self.__data.append(value)


class GridMaker(PointCloud):
    def __init__(self, pcd_topic, oversample=False, grid_size=40):
        super(GridMaker, self).__init__(pcd_topic, oversample)

        self.grid_size = grid_size
        self.grids = None

    def gridFilter(self, grids):
        ids = []

        for i in range(len(grids)):
            if len(grids[i].data) < 100:
                ids.append(i)

        for i in range(len(grids)):
            for j in ids:
                if i == j:
                    grids[i] = None

        return [x for x in grids if x is not None]

    def makeGrid(self):
        if self.data is None:
            rospy.logwarn("No Global Map")
            return

        grids = []
        x_min = y_min = float("inf")
        x_max = y_max = -1.0 * float("inf")

        # get x/y min/max
        for li in self.data:
            # li == [x, y, z]
            x = li[0]
            y = li[1]

            if x < x_min:
                x_min = x

            if y < y_min:
                y_min = y

            if x > x_max:
                x_max = x

            if y > y_max:
                y_max = y

        x_plus_iter = int(ceil(x_max / self.grid_size))
        x_minus_iter = int(floor(abs(x_min) / self.grid_size))

        y_plus_iter = int(ceil(y_max / self.grid_size))
        y_minus_iter = int(floor(abs(y_min) / self.grid_size))

        print(x_min, x_max, y_min, y_max)

        # create grids
        for x in range(-x_minus_iter, x_plus_iter):
            for y in range(-y_minus_iter, y_plus_iter):
                x_floor = x * self.grid_size
                x_ceil = (x + 1) * self.grid_size

                y_floor = y * self.grid_size
                y_ceil = (y + 1) * self.grid_size

                new_grid = Grid(xrange=(x_floor, x_ceil),
                                yrange=(y_floor, y_ceil))
                grids.append(new_grid)

        for d in self.data:
            # d == [x, y, z]
            x = d[0]
            y = d[1]

            for grid in grids:
                # grid = Grid()
                if grid.xrange[0] < x and x < grid.xrange[1] and grid.yrange[0] < y and y < grid.yrange[1]:
                    grid.dataAppender(d)
                    break

        self.grids = self.gridFilter(grids)


if __name__ == "__main__":
    rospy.init_node("global_map_grid")

    pcd = GridMaker(pcd_topic="cloud_pcd", oversample=False, grid_size=80)

    pub = rospy.Publisher("grid_publisher", PointCloud2, queue_size=5)

    tr = rospy.Rate(10)
    while not rospy.is_shutdown():
        if pcd.data is not None:
            pcd.makeGrid()
            break

        tr.sleep()

    r = rospy.Rate(hz=5)
    while not rospy.is_shutdown():

        for i in pcd.grids:
            try:
                point_cloud = array_to_xyz_pointcloud2f(
                    cloud_arr=i.data, stamp=rospy.Time.now(), frame_id="map")
                pub.publish(point_cloud)
                rospy.loginfo("Publishing..")

            except Exception as ex:
                rospy.logwarn(ex)

            r.sleep()

        r.sleep()
