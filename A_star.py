import cv2
import numpy as np
import time
# from __future__ import print_function
# from PIL import Image as im
import matplotlib.pyplot as plt
from robolab_turtlebot import Turtlebot, Rate, get_time


def heuristic(pos1, pos2):
    """
    Calculate euclidean distance, heuristic function
    """
    return ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5


def action_recognition(s_0, s_1):
    diff_x = s_1[1] - s_0[1]
    diff_y = s_1[0] - s_0[0]
    return {
        (0, 0): None,
        (0, 1): 0,
        (1, 1): 1,
        (1, 0): 2,
        (1, -1): 3,
        (0, -1): 4,
        (-1, -1): 5,
        (-1, 0): 6,
        (-1, 1): 7
    }[(diff_x, -diff_y)]


def assign_actions_to_path_point(path_given):
    actions_list = []
    for square_index in range(len(path_given) - 1):
        actions_list.append([path_given[square_index], action_recognition(path_given[square_index],
                                                                          path_given[square_index+1])])
    actions_list.append([path_given[-1], 8])  # Goal state = 8, no action
    return actions_list


class Problem:

    def __init__(self, map_given, goal):
        self.map = map_given
        self.goal = goal
        self.start = (99, 49)

    def find_path(self):
        """
        Method to find path in a graph using A* algorithm
        Returns a path_section as a list of positions [(x1, y1), (x2, y2), ... ].
        """
        path_exists = 0  # needed to know if we should return path or None
        memory = {}  # "memory" for visited nodes, will include position of parent and g score of node
        goal = self.goal  # goal position (x, y)
        start = self.start  # initial state (x, y)
        memory[start] = [None, 0]  # setting parent to None and g score of starting node to 0
        open_list = [start]  # list of opened nodes (doesn't include "closed"), adding starting node to opened ones
        min_f_pos = None

        while len(open_list):  # iterate while not empty
            min_f = 1000000  # minimal reached f score during selection, at start initializing to some big value
            min_f_pos = None  # position of node with minimal f score
            min_i = 0

            for i in range(len(open_list)):
                #   selecting the node with minimal f score
                if memory[open_list[i]][1] + heuristic(open_list[i], goal) < min_f:
                    min_f = memory[open_list[i]][1] + heuristic(open_list[i], goal)
                    min_f_pos = open_list[i]
                    min_i = i

            if min_f_pos == goal:  # break the loop when the goal position is reached
                print("goal reached")
                path_exists = 1
                break

            new_positions = self.expand(memory, min_f_pos)  # [[(x1, y1), cost], [(x2, y2), cost], ... ]

            open_list.pop(min_i)  # removing closed value

            for candidate in new_positions:  # iterating through children
                candidate_pos = candidate[0]
                candidate_step_cost = candidate[1]
                if candidate_pos in memory:
                    if memory[min_f_pos][1] + candidate_step_cost < memory[candidate_pos][1]:
                        #   if f score of candidate is lower than previous f score of this node, select the new
                        #   path by changing the parent to current one
                        memory[candidate_pos] = [min_f_pos, memory[min_f_pos][1] + candidate_step_cost]
                else:
                    #   if it's not in memory yet, add it
                    open_list.append(candidate_pos)
                    memory[candidate_pos] = [min_f_pos, memory[min_f_pos][1] + candidate_step_cost]

        if not path_exists:
            print('Path doesnt exist.')
            return None
        path = []
        while min_f_pos is not None:  # iterating back through the whole path and saving the nodes
            path.append(min_f_pos)
            min_f_pos = memory[min_f_pos][0]

        # Need to reverse the array because we started from end to start
        # For every pixel assigns action that will lead to the next pixel. For [(99, 49)], [(98, 49)] action is 0 (up).
        return assign_actions_to_path_point(path[::-1])

    def expand(self, visited_positions, pos):
        ret = []
        moves = [[-1, 0],  # go up
                 [0, -1],  # go left
                 [1, 0],  # go down
                 [0, 1],  # go right
                 [-1, -1],  # go left down
                 [1, 1],  # go right up
                 [-1, 1],  # go left up
                 [1, -1]  # go right down
                 ]
        for move in moves:
            if abs(move[0] + move[1]) != 1:
                price = 1.414
            else:
                price = 1
            x = pos[0] + move[0]
            y = pos[1] + move[1]
            if (x, y) not in visited_positions and 0 < x < 100 and 0 < y < 100 and self.map[x][y] != 1:
                ret.append(((x, y), price))
        return ret

