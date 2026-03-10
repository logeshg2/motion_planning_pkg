#!/usr/bin/env python3

"""Pinocchio + Robot arm utils"""


import pinocchio
import numpy as np
from scipy.spatial.transform import Rotation


def add_self_collision(collision_model, srdf_path: str):
    pass


def get_random_config(model):
    """Function to generate random joint configuration (under joint limits)"""
    return pinocchio.randomConfiguration(model, model.lowerPositionLimit, model.upperPositionLimit)


def isInCollision(model, collision_model, config_q):
    """Function to check all collision pairs for the given robot config"""
    
    data = model.createData()
    collision_data = collision_model.createData()

    # check collision (for Pinocchio collision pairs)
    stop_at_first_collision = True  # for fast computation
    pinocchio.computeCollisions(model, data, collision_model, collision_data, config_q, stop_at_first_collision)

    return np.any([cr.isCollision() for cr in collision_data.collisionResults])


def isCollision_free(node1, node2, model, collision_model, step=0.01):
    """Function to check the line segment between node1 and node2 in collision region or not"""
    
    # unit vector (between two nodes)
    v = node2.q - node1.q
    length = np.linalg.norm(v)
    length = max(length, 1e-6)
    u_v = (v / length)
    
    steps = int(length / step)

    if (steps < 2):
        return True

    step = length / (steps - 1)

    # check points in the line segment between node1 and node2
    for idx in range(steps):
        temp_q = node1.q + (u_v * (step * idx))
        if (isInCollision(model, collision_model, temp_q)):
            return False
    
    return True



