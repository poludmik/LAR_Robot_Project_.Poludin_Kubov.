"""
Best worlds for testing:
$ roslaunch lar lar_20n_bringup.launch gui:=true world_file:=track_2021T1_B_01.world
$ roslaunch lar lar_20n_bringup.launch gui:=true world_file:=track_2021T1_G_01.world
$ roslaunch lar lar_20n_bringup.launch gui:=true world_file:=track_2021T1_R_01.world

Run with argument "red" "green" or "blue"
"""

#!/usr/bin/env python

from __future__ import print_function

import cv2

import numpy as np

import sys

import image_processing_functions as impf

from robolab_turtlebot import Turtlebot

import image_processing_functions as impf
import moving_functions as mf

MOVE = 1
ROTATE = 2

linear_vel = 0.2
angular_vel = 0.3

WINDOW = 'obstacles'
WINDOW2 = 'rgb'

running = True
active = True


def click(vent, x, y, flags, param):
    global active
    active = not active
    print(active)


def main():
    if len(sys.argv) == 1:
        print("No argument")
        exit(2)
    argument = sys.argv[1]
    if argument != "red" and argument != "green" and argument != "blue":
        print("Incorrect color")
        exit(3)
    elif argument == "red":
        needed_color = 100
    elif argument == "green":
        needed_color = 200
    else: # blue
        needed_color = 50


    turtle = Turtlebot(pc=True, rgb=True)
    # turtle = Turtlebot(rgb=True)
    print('Waiting for point cloud ...')
    turtle.wait_for_point_cloud()
    direction = None
    print('First point cloud recieved ...')
    print('Waiting for rgb camera')
    turtle.wait_for_rgb_image()

    # cv2.namedWindow(WINDOW)
    cv2.namedWindow(WINDOW2)
    cv2.setMouseCallback(WINDOW, click)

    while not turtle.is_shutting_down():
        # # get point cloud
        # pc = turtle.get_point_cloud()
        # get rgb image
        rgb = turtle.get_rgb_image()

        thresh = impf.get_threshold_rgb(rgb)

        output = cv2.connectedComponentsWithStats(thresh)
        (numLabels, labels, stats, centroids) = output
        print("Labels count: ", numLabels)
        for i in range(numLabels):
            area = stats[i, cv2.CC_STAT_AREA]
            (cX, cY) = centroids[i]                         # get coords of column
            if impf.is_region_a_column(stats[i]) and thresh[int(cY)][int(cX)] == needed_color:
                print("Found", cX, cY)
                if cX > len(rgb[0])/2+30:                   # if it's right from the centre axis
                    mf.move_rotate(turtle, 0.1, -angular_vel / 3)
                elif cX < len(rgb[0])/2-30:                 # if left from the centre
                    mf.move_rotate(turtle, 0.1, angular_vel / 3)
                else:
                    mf.move_forward(turtle, 5, linear_vel)
            else:
                mf.move_rotate(turtle, 0.1, angular_vel/6)

        # show image
        cv2.imshow(WINDOW2, thresh)
        cv2.waitKey(1)

        # if pc is None:
        #     print('No point cloud')
        #     continue
        #
        # # mask out floor points
        # mask = pc[:, :, 1] < 0.2

        # # mask point too far
        # mask = np.logical_and(mask, pc[:, :, 2] < 3.0)

        # # empty image
        # image = np.zeros(mask.shape)
        #
        # # assign depth i.e. distance to image
        # image[mask] = np.int8(pc[:, :, 2][mask] / 3.0 * 255)
        # im_color = cv2.applyColorMap(255 - image.astype(np.uint8),
        #                              cv2.COLORMAP_JET)
        #
        # # show image
        # cv2.imshow(WINDOW, im_color)
        # cv2.waitKey(1)

        # # check obstacle
        # mask = np.logical_and(mask, pc[:, :, 1] > -0.2)
        # data = np.sort(pc[:, :, 2][mask])
        #
        # state = MOVE
        # if data.size > 50:
        #     dist = np.percentile(data, 10)
        #     if dist < 0.6:
        #         state = ROTATE



if __name__ == '__main__':
    main()
