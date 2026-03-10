#!/usr/bin/env python3

import numpy as np

from mpkg.utils.pin_utils import isCollision_free


class Node:
    def __init__(self, q, parent=None, cost=0):
        self.q = q
        self.parent = parent
        self.cost = cost


def steer(near_node, rand_node, steer_dist, lowerLimit, upperLimit):
    """Function to perform steering operation based on near and random node"""
    
    # unit vector (between two nodes)
    v = rand_node.q - near_node.q
    u_v = (v / np.linalg.norm(v))           # unit vector of - v

    distance = min(np.linalg.norm(v), steer_dist)

    # scale it up for the distance (steer distance)
    scaled_q = u_v * distance                     # this is from the origin like (but we need from near_node point)
    # scaled_q from near_node
    new_q = near_node.q + scaled_q

    # clamp the new_q
    new_q = np.clip(new_q, lowerLimit, upperLimit)
    
    # compute cost between new_node and near_node (Parent Cost + Dist cost)
    cost = near_node.cost + np.linalg.norm(new_q - near_node.q)

    # return new node
    return Node(new_q, cost=cost)


def nearest(tree, rand_node):
    """Function to find the nearest node in the explored tree"""

    minDist = np.inf
    nearNode = None
    for node in tree:
        # euclidian distance (L-2 norm)
        dist = np.linalg.norm(rand_node.q - node.q)

        if (dist == 0.0):
            return None

        if (dist < minDist):
            minDist = dist
            nearNode = node    
    return nearNode


def upDateTree(tree, neigh_radius, model, collision_model):
    """Function perform Rewiring and Find best neighbour node in the node"""

    # last node is the new node (for which optimization is performed)
    newNode = tree[-1]
    # get all neighbour nodes
    neigh_nodes = []
    for node in tree:
        dist = np.linalg.norm(newNode.q - node.q)
        if (dist == 0.0):
            continue        # omit itself
        if dist <= neigh_radius:
            neigh_nodes.append(node)
    
    # choose best parent (based on cost)
    best_parent = None
    best_cost = np.inf
    for node in neigh_nodes:
        dist = np.linalg.norm(newNode.q - node.q)
        temp_cost = node.cost + dist
        if (temp_cost < best_cost) and (isCollision_free(node, newNode, model, collision_model)):
            best_cost = temp_cost
            best_parent = node
    # update newNode's parent based on cost
    if (best_parent is not None):
        newNode.cost = best_cost
        newNode.parent = best_parent
    
    # help neighbours using newNode (cheap cost basis)
    # updates neighbour nodes as well
    for node in neigh_nodes:
        dist = np.linalg.norm(newNode.q - node.q)
        new_cost = newNode.cost + dist
        if (new_cost < node.cost) and (isCollision_free(newNode, node, model, collision_model)):
            node.cost = new_cost
            node.parent = newNode


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


def shortcut(path, model, collision_model, num_itr=100):
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
        if (isCollision_free(low_node, high_node, model, collision_model)):
            path = path[:low_idx+1] + path[high_idx:]
            print("Path shortcut applied!")
    
    return path


def getPathNodesFromTree(tree):
    """Function to extract path nodes from entire tree"""

    # get the path from start node to goal node
    path = []
    goalNode = tree[-1]                 # last node inserted in goal node
    path.append(goalNode)
    startReached = False

    while (not startReached):
        tempNode = path[-1]
        path.append(tempNode.parent)

        if (tempNode.parent is None):
            startReached = True

    path.reverse()                      # start to goal
    return path


def pathNode2PathConfig(path):
    """Function extract path node to path config (only joint values)"""
    
    pathConfig = []
    for node in path:
        pathConfig.append(node.q)

    return pathConfig


def printPath(path):
    """Function to print path (all joint configs)"""

    for node in path:
        print(node.q)