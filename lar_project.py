# !/usr/bin/env python

from __future__ import print_function

import cv2

import numpy as np

import A_star as astar

import sys

import copy

import time

import math

from robolab_turtlebot import Turtlebot

import image_processing_functions as impf

import moving_functions as mf

MOVE = 1
ROTATE = 2

linear_vel = 0.2
angular_vel = 0.3

MAP_SIZE = 100  # [cells]
CELL_SIZE = 5  # [cm]

WINDOW = 'obstacles'
WINDOW2 = 'rgb'

running = True
active = True


def click(vent, x, y, flags, param):
    global active
    active = not active
    print(active)


def get_color_from_argv(argv):
    if len(argv) == 1:
        print("No argument")
        exit(2)
    argument = sys.argv[1]
    needed_color = 0
    if argument != "red" and argument != "green" and argument != "blue":
        print("Incorrect color")
        exit(3)
    elif argument == "red":
        needed_color = 100
    elif argument == "green":
        needed_color = 200
    else:  # blue
        needed_color = 50
    return needed_color


def rotate_towards_target(robot, output, image, needed_color):
    print("Rotation start")
    (numLabels, labels, stats, centroids) = output
    for i in range(numLabels):
        area = stats[i, cv2.CC_STAT_AREA]
        (cX, cY) = centroids[i]  # get coords of column
        if impf.is_region_a_column(stats[i]) and image[int(cY)][int(cX)] == needed_color:
            print("Found", cX, cY)
            if cX > len(image[0]) / 2 + 30:  # if it's right from the centre axis
                mf.move_rotate(robot, 0.01, -angular_vel / 6)
            elif cX < len(image[0]) / 2 - 30:  # if left from the centre
                mf.move_rotate(robot, 0.01, angular_vel / 6)
            else:
                print("Rotated, returning.")
                return int(cX), int(cY)
        else:
            mf.move_rotate(robot, 0.1, angular_vel / 6)


def set_column_on_map(map, coords, is_goal):
    if math.isnan(coords[0]) or math.isnan(coords[2]):
        return map
    y = int(coords[2] * 100 / CELL_SIZE)
    x = int(MAP_SIZE / 2 + coords[0] * 100 / CELL_SIZE)
    target = None
    if is_goal:
        map[MAP_SIZE - y][x] = 2
        target = (MAP_SIZE - y, x)
    else:
        map[MAP_SIZE - y][x] = 1
    # print(x, y)
    return map, target


def draw_restricted_zone(map_array):
    r = 5
    map2 = copy.deepcopy(map_array)
    count_not_goals = 0
    for i in range(MAP_SIZE):
        for j in range(MAP_SIZE):
            if map_array[i][j] == 1:
                count_not_goals += 1
                for x in range(-r + i, r + i):
                    for y in range(-r + j, r + j):
                        if 0 < x < 100 and 0 < y < 100:
                            map2[x][y] = 1
    return map2


def main():
    needed_color = get_color_from_argv(sys.argv)

    map_array = np.zeros((MAP_SIZE, MAP_SIZE))

    state = "START"

    target = target_local = None

    turtle = Turtlebot(pc=True, rgb=True)
    print('Waiting for point cloud ...')
    turtle.wait_for_point_cloud()
    print('First point cloud recieved ...')
    print('Waiting for rgb camera')
    turtle.wait_for_rgb_image()
    print('First RGB received')

    # cv2.namedWindow(WINDOW)
    cv2.namedWindow(WINDOW2)
    cv2.setMouseCallback(WINDOW, click)

    count_times_to_print = 0

    while not turtle.is_shutting_down():

        # get point cloud
        pc = turtle.get_point_cloud()
        # get rgb image
        rgb = turtle.get_rgb_image()

        thresh = impf.get_threshold_rgb(rgb)
        output = cv2.connectedComponentsWithStats(thresh)
        (numLabels, labels, stats, centroids) = output

        if count_times_to_print % 5 == 0:
            print("Labels count: ", numLabels)
            print('Current state:', state)
            count_times_to_print = 0
        count_times_to_print += 1

        if state == "START":
            if rotate_towards_target(turtle, output, thresh, needed_color) is not None:
                rotated = True
                state = "ROTATED"
                time.sleep(2)  # needed for distance readings to settle
                continue
            else:
                continue

        # show image
        cv2.imshow(WINDOW2, thresh)
        cv2.waitKey(1)

        if pc is None:
            print('No point cloud')
            continue

        if state == "ROTATED":
            (numLabels, labels, stats, centroids) = output
            for i in range(numLabels):
                (cX, cY) = centroids[i]  # get coords of column
                if impf.is_region_a_column(stats[i]):
                    if thresh[int(cY)][int(cX)] == needed_color:
                        map_array, target = set_column_on_map(map_array, pc[int(cY), int(cX)], True)
                        target_local = target  # sometimes was broken,
                        # because the goal is not always the last one to set on the map
                    else:
                        map_array, target = set_column_on_map(map_array, pc[int(cY), int(cX)], False)
            if target_local is not None:
                target = target_local
                state = "COLUMNS_SET"

        if state == "COLUMNS_SET":
            map_array = draw_restricted_zone(map_array)
            state = "RESTRICTIONS_DRAWN"

        if state == "RESTRICTIONS_DRAWN":
            problem = astar.Problem(map_array, target)
            path = problem.find_path()
            print('path =', path)
            if path is not None:
                state = "PATH_FOUND"
            for element in path:
                map_array[element[0][0]][element[0][1]] = 3
                pass

        mask = pc[:, :, 1] < 0.2  # mask out floor points
        mask = np.logical_and(mask, pc[:, :, 2] < 4.0)  # mask point too far
        image = np.zeros(mask.shape)  # empty image
        image[mask] = np.int8(pc[:, :, 2][mask] / 3.0 * 255)  # assign depth i.e. distance to image

        # im_color = cv2.applyColorMap(255 - image.astype(np.uint8),
        #                              cv2.COLORMAP_JET)
        # show image
        # cv2.imshow(WINDOW, im_color)

        # resize of the map 5x times
        map_to_show = cv2.resize(map_array, dsize=(MAP_SIZE*5, MAP_SIZE*5), interpolation=cv2.INTER_CUBIC)
        cv2.imshow(WINDOW, map_to_show)
        cv2.waitKey(1)


if __name__ == '__main__':
    main()

