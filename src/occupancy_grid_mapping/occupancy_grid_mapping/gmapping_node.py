#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import cv2
from time import perf_counter
import sys
import threading
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from .submodules.bresenham import *
from .submodules.utils import *
from .submodules.grid_map import *

MAPS_PATH = 'maps/'

class GmappingClass(Node):

    def __init__(self):
        super().__init__('gmapping_node')
        self.distances = np.array([])
        self.angles = np.array([])
        self.information = np.array([])
        self.x_odom, self.y_odom, self.theta_odom, self.range_max = 0.0, 0.0, 0.0, 0.0

        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, qos_profile=qos_profile_sensor_data)

    def scan_callback(self, msgScan):
        """
        Convert LaserScan msg to array
        """

        self.range_max = msgScan.range_max
        self.distances, self.angles, self.information = lidar_scan(msgScan)
        # distances in [m], angles in [radians], information [0-1] (probability)

    def odom_callback(self, msgOdom):
        """
        Get (x,y) coordinates from Odometry msg in [m]
        """
        self.x_odom = msgOdom.pose.pose.position.x
        self.y_odom = msgOdom.pose.pose.position.y
        orientation_q = msgOdom.pose.pose.orientation
        self.theta_odom = transform_orientation(orientation_q)

def main():

    P_prior = 0.5  # Prior occupancy probability
    P_occ = 0.9	    # Probability that cell is occupied with total confidence
    P_free = 0.3  # Probability that cell is free with total confidence

    RESOLUTION = 0.03  # Grid resolution in [m]

    MAP_NAME = 'world'  # map name without extension

    map_x_lim = [-10, 10]
    map_y_lim = [-10, 10]

    rclpy.init(args=sys.argv)
    node = GmappingClass()
    node.get_logger().info('Created gmapping_node')
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(20)

    print("trying .... grid creation")
    # Create grid map
    gridMap = GridMap(X_lim=map_x_lim,
                      Y_lim=map_y_lim,
                      resolution=RESOLUTION,
                      p=P_prior)

    print("grid created")

    # Init time
    t_start = perf_counter()
    sim_time = 0
    step = 0

    try:
        while rclpy.ok():
            # waiting until msg is received is missing
            distances, angles, information = node.distances, node.angles, node.information
            x_odom, y_odom, theta_odom = node.x_odom, node.y_odom, node.theta_odom

            print('distances:\n', distances, '\nangles:\n', angles, '\nx_odom:\n',
                  x_odom, '\ny_odom:\n', y_odom, '\ntheta_odom:\n', theta_odom)
            distances_x, distances_y = lidar_scan_xy(
                distances, angles, x_odom, y_odom, theta_odom)

            # x1 and y1 for Bresenham's algorithm
            x1, y1 = gridMap.discretize(x_odom, y_odom)

            # for BGR image of the grid map
            X2 = []
            Y2 = []

            for (dist_x, dist_y, dist) in zip(distances_x, distances_y, distances):

                # x2 and y2 for Bresenham's algorithm
                x2, y2 = gridMap.discretize(dist_x, dist_y)

                # draw a discrete line of free pixels, [robot position -> laser hit spot)
                for (x_bres, y_bres) in bresenham(gridMap, x1, y1, x2, y2):

                    gridMap.update(x=x_bres, y=y_bres, p=P_free)

                # mark laser hit spot as ocuppied (if exists)
                if dist < node.range_max:

                    gridMap.update(x=x2, y=y2, p=P_occ)

                # for BGR image of the grid map
                X2.append(x2)
                Y2.append(y2)

            # converting grip map to BGR image
            bgr_image = gridMap.to_BGR_image()

            # marking robot position with blue pixel value
            set_pixel_color(bgr_image, x1, y1, 'BLUE')

            # marking neighbouring pixels with blue pixel value
            for (x, y) in gridMap.find_neighbours(x1, y1):
                set_pixel_color(bgr_image, x, y, 'BLUE')

            # marking laser hit spots with green value
            for (x, y) in zip(X2, Y2):
                set_pixel_color(bgr_image, x, y, 'GREEN')
            print("resizing image")
            resized_image = cv2.resize(src=bgr_image,
                                       dsize=(500, 500),
                                       interpolation=cv2.INTER_AREA)
            print("rotating image")
            rotated_image = cv2.rotate(src=resized_image,
                                       rotateCode=cv2.ROTATE_90_COUNTERCLOCKWISE)

            cv2.imshow("Grid map", rotated_image)
            cv2.waitKey(1)

            # Calculate step time in [s]
            t_step = perf_counter()
            step_time = t_step - t_start
            sim_time += step_time
            t_start = t_step
            step += 1

            print('Step %d ==> %d [ms]' % (step, step_time * 1000))

            rate.sleep()
            # print("slept")

    # try:
    # 	while rclpy.ok():
    # 		rate.sleep()
    # except KeyboardInterrupt:
    # 	pass

    # try:
    # 	while rclpy.ok():
    # 		rclpy.spin(node)
    # except (KeyboardInterrupt):
    # 	pass
    except Exception as e:
        print("Exception is: ", e)
        print('\r\nSIMULATION TERMINATED!')
        print('\nSimulation time: %.2f [s]' % sim_time)
        print('Average step time: %d [ms]' % (sim_time * 1000 / step))
        print('Frames per second: %.1f' % (step / sim_time))

        # Saving Grid Map
        resized_image = cv2.resize(src=gridMap.to_BGR_image(),
                                   dsize=(500, 500),
                                   interpolation=cv2.INTER_AREA)

        rotated_image = cv2.rotate(src=resized_image,
                                   rotateCode=cv2.ROTATE_90_COUNTERCLOCKWISE)

        flag_1 = cv2.imwrite(img=rotated_image * 255.0,
                             filename=MAPS_PATH + '/' + MAP_NAME + '_grid_map_TEST.png')

        # Calculating Maximum likelihood estimate of the map
        gridMap.calc_MLE()

        # Saving MLE of the Grid Map
        resized_image_MLE = cv2.resize(src=gridMap.to_BGR_image(),
                                       dsize=(500, 500),
                                       interpolation=cv2.INTER_AREA)

        rotated_image_MLE = cv2.rotate(src=resized_image_MLE,
                                       rotateCode=cv2.ROTATE_90_COUNTERCLOCKWISE)

        flag_2 = cv2.imwrite(img=rotated_image_MLE * 255.0,
                             filename=MAPS_PATH + '/' + MAP_NAME + '_grid_map_TEST_mle.png')

        if flag_1 and flag_2:
            print('\nGrid map successfully saved!\n')

        if cv2.waitKey(0) == 27:
            cv2.destroyAllWindows()

        pass

    rclpy.shutdown()
    thread.join()
    # finally:
    # 	node.destroy_node()
    # 	rclpy.try_shutdown()


if __name__ == '__main__':
    main()
