#!/usr/bin/env python3

import coal
import time
import pinocchio
import numpy as np
import matplotlib.pyplot as plt
from pinocchio.visualize import MeshcatVisualizer



## Helper Functions

class Node:
    def __init__(self, q, parent=None, cost=0):
        self.q = q
        self.parent = parent
        self.cost = cost

def visualizeAndWait(q_config, wait=True):
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True)

    viz.loadViewerModel()

    viz.display(q_config)
    viz.displayVisuals(True)
    viz.displayCollisions(True)

    if (wait):
        print("Enter to continue")
        input()

def get_random_config():
    return pinocchio.randomConfiguration(model, model.lowerPositionLimit, model.upperPositionLimit)

def nearest(tree, rand_node):
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

def steer(near_node, rand_node):
    # unit vector (between two nodes)
    v = rand_node.q - near_node.q
    u_v = (v / np.linalg.norm(v))           # unit vector of - v

    distance = min(np.linalg.norm(v), steer_dist)

    # scale it up for the distance (steer distance)
    scaled_q = u_v * distance                     # this is from the origin like (but we need from near_node point)
    # scaled_q from near_node
    new_q = near_node.q + scaled_q

    # clamp the new_q
    new_q = np.clip(new_q, model.lowerPositionLimit, model.upperPositionLimit)
    
    # compute cost between new_node and near_node (Parent Cost + Dist cost)
    cost = near_node.cost + np.linalg.norm(new_q - near_node.q)

    # return new node
    return Node(new_q, cost=cost)

def isCollision_free(node1, node2, step=0.01):
    # unit vector (between two nodes)
    v = node2.q - node1.q
    length = np.linalg.norm(v)
    u_v = (v / length)
    
    steps = int(length / step)

    if (steps < 1):
        return True

    step = length / (steps - 1)

    # check points in the line segment between node1 and node2
    for idx in range(steps):
        temp_q = node1.q + (u_v * (step * idx))
        if (is_collision(Node(temp_q))):
            return False
    
    return True

def is_collision(new_node):
    collision_data = collision_model.createData()
    # check collision (using Pinocchio collision pairs)
    stop_at_first_collision = True  # for fast computation
    pinocchio.computeCollisions(model, data, collision_model, collision_data, new_node.q, stop_at_first_collision)

    return np.any([cr.isCollision() for cr in collision_data.collisionResults])

def upDateTree():
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
        if (temp_cost < best_cost) and (isCollision_free(node, newNode)):
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
        if (new_cost < node.cost) and (isCollision_free(newNode, node)):
            node.cost = new_cost
            node.parent = newNode

def printPath(path):
    for node in path:
        print(node.q)

def discretize_joint_position(path_node, step=0.01):
    """
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
        u_v = (v / length)
        steps = int(length / step)
        if (steps >= 1):
            step = length / (steps - 1)
            for idx in range(steps):
                dis_path.append(q1 + (u_v * (step * idx)))

        dis_path.append(q2)

    return dis_path

##

# FANUC robot model
model, collision_model, visual_model = pinocchio.buildModelsFromUrdf("/home/logesh/fanuc_ws/src/fanuc_ros2_drivers/src/fanuc_description/urdf/lrmate200id4s.urdf")
srdf_model_path = "/home/logesh/fanuc_ws/src/fanuc_ros2_drivers/src/fanuc_description/urdf/lrmate200id4s.srdf"
data = pinocchio.createDatas(model)[0]

eeFrameId = model.getFrameId("tool0")
# collision pairs setup
collision_model.addAllCollisionPairs()
pinocchio.removeCollisionPairs(model, collision_model, srdf_model_path)     # remove adjacent joints from collision pairs
###
# custom object
cube_geom = pinocchio.GeometryObject(
    "cube",
    0,                 # parent joint (0 = world)
    pinocchio.SE3(np.eye(3), np.array([0.2, 0.2, 0.5])),
    coal.Box(0.15, 0.15, 0.15)
)
cube_geom.meshColor = np.array([0.5, 0.5, 0.5, 0.5])
cube_id = collision_model.addGeometryObject(cube_geom)
visual_model.addGeometryObject(cube_geom)
# add cube to the collision pairs
for i in range(cube_id):
    collision_model.addCollisionPair(
        pinocchio.CollisionPair(i, cube_id)
    )
###

# by default pinocchio uses save seed every time of start (which is not what we needed) - NOTE: can still set to particular seed
pinocchio.seed(42)     # int(time.time())

# RRT* (planning options or parameters)
K = 5000
neigh_radius = 0.2      # 0.2 radian radius in joint configuration space
steer_dist = 0.3        # for now keeping steering distance as constant
goal_threshold = 0.5
goal_bias = 0.30    
reachedGoal = False

# start and goal config
start_q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
goal_q = np.array([1.0, 0.467, -0.204, 0.0, -1.094, 0.0])
goal_q = np.array([1.5, 0.7, 0.0, 0.0, -1.5, 0.0])

# tree
tree = []
tree.append(Node(start_q))

# check start and goal config in collision
if (is_collision(Node(start_q))):
    print("Start config is in collision")
    exit(0)
if (is_collision(Node(goal_q))):
    print("Goal config is in collision")
    exit(0)

start_time = time.perf_counter()
# search
for idx in range(K):
    # goal biasing
    if np.random.random() < goal_bias:      # every 10% of time (the target node will be the random node)
        rand_node = Node(goal_q)
    else:
        # generate random joint configuration
        rand_node = Node(get_random_config())
    # find the nearest of node to the randomly generated node
    near_node = nearest(tree, rand_node)

    if (near_node is None):
        print("Random node failed (no near node found in the tree)")
        continue

    # steer towards the randomly generated node
    new_node = steer(near_node, rand_node)

    # check collision
    if (not isCollision_free(near_node, new_node)):
        print("New node under collision - going back")
        continue

    # update parent for back tracking (path finding)
    new_node.parent = near_node
    tree.append(new_node)

    # RRT* additional steps
    upDateTree()

    # check closness of new node to goal node (using thresholding)
    goal_dist = np.linalg.norm(np.array(goal_q) - new_node.q)

    if (goal_dist <= goal_threshold):
        # if distance is minimal (connect to the goal node)
        goal_cost = new_node.cost + goal_dist
        goal_node = Node(goal_q, new_node, goal_cost)
        tree.append(goal_node)

        reachedGoal = True
        print("Goal Position reached!")
        break

if (not reachedGoal):
    print("Unable to reach goal this time!!!")
    exit(0)

# get the path from start node to goal node
path = []
goalNode = tree[-1]     # last node inserted in goal node
path.append(goalNode)
startReached = False

while (not startReached):
    tempNode = path[-1]
    path.append(tempNode.parent)

    if (np.all(np.equal(tempNode.parent.q, np.array(start_q)))):
        startReached = True

path.reverse()

print("\nComputed Path\n")
printPath(path)

print(f"\nTotal Cost: {path[-1].cost}")
print(f"\nTime taken: {time.perf_counter() - start_time} sec\n")

# plotPaths(path, tree)

###
# visualize path
viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)
viz.loadViewerModel()

viz.display(start_q)
time.sleep(0.05)

"""
input("Enter to continue simulation:")
for q_p in path:
    viz.display(q_p.q)
    time.sleep(0.05)
"""
    
# discretize the path

discretized_path = discretize_joint_position(path)
input("Enter to continue simulation:")
for q_p in discretized_path:
    viz.display(q_p)
    time.sleep(0.05)

###