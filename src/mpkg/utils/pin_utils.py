#!/usr/bin/env python3

"""Pinocchio + Robot arm utils"""


import pinocchio
import numpy as np
from scipy.spatial.transform import Rotation
from pinocchio.visualize import MeshcatVisualizer


def add_self_collision(model, collision_model, srdf_path: str):
    """Function to add all self collision pairs"""

    collision_model.addAllCollisionPairs()
    # remove adjacent and never collsion pairs
    pinocchio.removeCollisionPairs(model, collision_model, srdf_path)


def add_object_collision(geom_object, collision_model, visual_model):
    """Function to add custom geometry object ro visual and collision model + add them to collision pairs"""
    
    gemo_obj_id = collision_model.addGeometryObject(geom_object)
    visual_model.addGeometryObject(geom_object)
    
    # add cube to the collision pairs
    for idx in range(gemo_obj_id):
        collision_model.addCollisionPair(
            pinocchio.CollisionPair(idx, gemo_obj_id)
        )
    
    return gemo_obj_id


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


def visualizeAndWait(model, collision_model, visual_model, q_config, wait=True):
    """Simple visualizing function to visual the robot arm on a given joint config"""
    
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True)

    viz.loadViewerModel()

    viz.display(q_config)
    viz.displayVisuals(True)
    viz.displayCollisions(True)

    if (wait):
        print("Enter to continue")
        input()
