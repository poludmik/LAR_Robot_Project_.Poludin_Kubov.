import cv2
import numpy as np
import time
import image_processing_functions as impf
# from __future__ import print_function
# from PIL import Image as im
import matplotlib.pyplot as plt
from robolab_turtlebot import Turtlebot, Rate, get_time


def move_forward(robot, time_of_moving, speed):
    rate = Rate(10)
    t = get_time()
    while get_time() - t < time_of_moving:
        robot.cmd_velocity(linear=speed)
        rate.sleep()


def move_rotate(robot, time_of_moving, speed):
    rate = Rate(10)
    t = get_time()
    while get_time() - t < time_of_moving:
        robot.cmd_velocity(angular=speed)
        rate.sleep()
