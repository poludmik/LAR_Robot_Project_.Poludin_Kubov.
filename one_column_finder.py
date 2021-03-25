#!/usr/bin/env python

import cv2
import numpy as np
import time
import image_processing_functions as impf
import moving_functions as mf
# from __future__ import print_function
# from PIL import Image as im
import matplotlib.pyplot as plt
from robolab_turtlebot import Turtlebot, Rate, get_time


if __name__ == '__main__':
    robot = Turtlebot(pc=True, rgb=True, depth=True)
    got_hit = False
    adjusting_way = False

    print('Waiting for point cloud ...')
    robot.wait_for_point_cloud()
    point_cloud = robot.get_point_cloud()
    print('First point cloud received ...')

    while not robot.is_shutting_down() or got_hit:
        pc = robot.get_point_cloud()
        if pc is None:
            print('No point cloud')
            continue

        image = robot.get_rgb_image()
        depth = robot.get_depth_image()

        thrash = impf.get_threshold_rgb(image)
        number_of_regions, connected_image, parameters, centers = impf.get_connected_regions(thrash)
        dep_th = impf.get_threshold_depth(depth, 1.7)  # anything closer than 1.7 meters

        if number_of_regions <= 1:
            mf.move_rotate(robot, 1, 0.3)
            print('no regions found!')
        else:
            fig = plt.figure(figsize=(9, 9))
            plt.imshow(connected_image)
            plt.show()
            adjusting_way = True

        if adjusting_way:
            print('adjusting way! hura')
            print(centers[1])
            while 100 < centers[1][1] or centers[1][1] > 300:
                mf.move_rotate(robot, 0.5, 0.2)
                number_of_regions, connected_image, parameters, centers = impf.get_connected_regions(thrash)
            adjusting_way = False
