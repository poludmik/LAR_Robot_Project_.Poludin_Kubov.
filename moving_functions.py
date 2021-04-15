import cv2
import numpy as np
import time
import image_processing_functions as impf
# from __future__ import print_function
# from PIL import Image as im
import matplotlib.pyplot as plt
from robolab_turtlebot import Turtlebot, Rate, get_time
import math


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


def move_given_distance_in_cm(robot, distance_cm):
    rate = Rate(10)
    velocity = 0.05  # lower speed = better precision
    robot.reset_odometry()
    distance_in_m = float(distance_cm) / 100
    print('distance to go (m)=', distance_in_m, 'velocity=', velocity)
    travelled_distance = 0
    # print('traveled=', travelled_distance * 100)
    while travelled_distance < distance_in_m - 0.01:
        robot.cmd_velocity(linear=velocity)
        rate.sleep()
        x, y, _ = robot.get_odometry()
        travelled_distance = math.sqrt(x**2+y**2)
        # print('traveled=', travelled_distance * 100)
    time.sleep(0.1)
    return True


def rotate_given_angle_in_deg(robot, angle_deg):
    velocity = 7
    rate = Rate(10)
    if angle_deg > 0:
        angular_velocity_deg = velocity  # lower speed = better precision (i.e. 1)
    else:
        angular_velocity_deg = -velocity
    angular_velocity_rad = angular_velocity_deg * 0.0174532925
    angle_in_rad = 0.0174532925 * angle_deg
    print('angle to rotate (deg)=', angle_deg, 'angular velocity (deg/s)=', angular_velocity_deg)
    robot.reset_odometry()
    travelled_angle_rad = 0
    # print('traveled_deg=', travelled_angle_rad / 0.0174532925)
    while abs(travelled_angle_rad) < abs(angle_in_rad - 0.038):
        robot.cmd_velocity(angular=angular_velocity_rad)
        rate.sleep()
        _, _, travelled_angle_rad = robot.get_odometry()
        # print('traveled_deg=', travelled_angle_rad / 0.0174532925)
    return True


