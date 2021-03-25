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


MOVE = 1
ROTATE = 2

linear_vel = 1
angular_vel = 0.3

running = True
active = True

if __name__ == '__main__':

    turtle = Turtlebot(pc=True, rgb=True, depth=True)

    print('Waiting for point cloud ...')
    turtle.wait_for_point_cloud()
    point_cloud = turtle.get_point_cloud()
    print('First point cloud received ...')

    image = turtle.get_rgb_image()
    depth = turtle.get_depth_image()
    # K_rgb = turtle.get_rgb_K()

    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    fig = plt.figure(figsize=(9, 9))
    columns = 2
    rows = 2
    thrash = impf.get_threshold_rgb(image)
    out = impf.get_connected_regions(thrash)
    dep_th = impf.get_threshold_depth(depth, 1.7)

    # mf.move_forward(turtle, 1, 0.3)
    mf.move_rotate(turtle, 1, 0.3)

    # print(out[0])
    # print(out[2])
    # print(out[3])
    #
    # for i in range(out[0]):
    #     print(impf.is_region_a_column(out[2][i]))

    # arr = [image, thrash, out[1]]
    # for i in range(1, len(arr) + 1):
    #     fig.add_subplot(rows, columns, i)
    #     plt.imshow(arr[i - 1])
    # plt.show()
