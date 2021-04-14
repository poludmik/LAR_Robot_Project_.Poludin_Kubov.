import cv2
import numpy as np
import time
import image_processing_functions as impf
# from __future__ import print_function
# from PIL import Image as im
import matplotlib.pyplot as plt
from robolab_turtlebot import Turtlebot, Rate, get_time


def get_threshold_rgb(image_local):
    """
    On the returned image:
    BLUE color - value 50
    GREEN color - value 200
    RED color - value 100
    """
    hsv = cv2.cvtColor(image_local, cv2.COLOR_BGR2HSV)

    color_mask = cv2.inRange(hsv, (0, 125, 25), (20, 255, 255))
    _, th_red = cv2.threshold(color_mask, 127, 100, cv2.THRESH_BINARY)
    color_mask = cv2.inRange(hsv, (35, 125, 25), (100, 255, 255))
    _, th_green = cv2.threshold(color_mask, 127, 200, cv2.THRESH_BINARY)
    color_mask = cv2.inRange(hsv, (85, 125, 25), (140, 255, 255))
    _, th_blue = cv2.threshold(color_mask, 127, 50, cv2.THRESH_BINARY)
    return th_green + th_red + th_blue


def get_threshold_depth(depth_image, given_depth):
    """
    Threshold of the points closer than 1.5 m.
    """
    _, th_dep = cv2.threshold(depth_image, given_depth, 255, cv2.THRESH_BINARY_INV)
    return th_dep


def get_connected_regions(th_image):
    """
    :return: out[0] - total number of regions
             out[1] - image of regions with values of their index
             out[2] - parameters [most left point, most up point, width, height, area]
             out[3] - center of a region
    """
    return cv2.connectedComponentsWithStats(np.uint8(th_image))


def is_region_a_column(parameters):
    width = parameters[2]
    height = float(parameters[3])
    if 8.2 < (height / width) < 11:  # changed to 11, sometimes didn't recognise columns with 10.8
        return True
    else:
        return False


def get_index_of_any_column(number_of_regions, parameters):
    index_region = None
    for i in range(number_of_regions):
        if impf.is_region_a_column(parameters[i]):
            index_region = i
    return index_region


