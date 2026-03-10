#!/usr/bin/env python3

import numpy as np

from mpkg.utils.pin_utils import isCollision_free


class Node:
    def __init__(self, q, parent=None, cost=0):
        self.q = q
        self.parent = parent
        self.cost = cost


def discretize_joint_position(path_node, step=0.01):
    """
    Function to discretize the input path (linear interpolation)
    This function was inspired from PyRoboPlan project.
    """

    q_arr = []
    for node in path_node:
        q_arr.append(node.q)
    
    dis_path = []
    for i in range(1, len(q_arr)):
        q1 = q_arr[i-1]
        q2 = q_arr[i]
        dis_path.append(q1)

        # between two configs
        v = q2 - q1
        length = np.linalg.norm(v)
        length = max(length, 1e-6)
        u_v = (v / length)
        steps = int(length / step)
        if (steps >= 1):
            step = length / (steps - 1)
            for idx in range(steps):
                dis_path.append(q1 + (u_v * (step * idx)))

        dis_path.append(q2)

    return dis_path


def shortcut(path, num_itr=100):
    """Function to find shortcut path between nodes in the path (reduces jerky and long paths)"""

    if (len(path) < 3):
        return path
    
    for _ in range(num_itr):
        if (len(path) < 3):
            break

        # get random nodes from the path
        low_idx, high_idx = sorted(np.random.randint(0, len(path), size=2).tolist())
        # get the joint configs
        low_node, high_node = path[low_idx], path[high_idx]
        
        # check whether straight line between nodes are collision free
        if (isCollision_free(low_node, high_node)):
            path = path[:low_idx+1] + path[high_idx:]
            print("Path shortcut applied!")
    
    return path