#!/usr/bin/env python3

"""
This is an example implementation of RRT for path planning for 2D navigation with collision check.
"""

import time
import numpy as np
import matplotlib.pyplot as plt


## helpers ##
class Node:
    def __init__(self, coord, parent=None):
        self.coord = np.array(coord)
        self.parent = parent

def get_rand_coord():
    r = rng.choice(range(C_space.shape[0]))
    c = rng.choice(range(C_space.shape[1]))
    return (r, c)

def nearest(tree, rand_node):
    minDist = np.inf
    nearNode = None
    for node in tree:
        # euclidian distance (L-2 norm)
        dist = np.linalg.norm(rand_node.coord - node.coord)

        if (dist == 0.0):
            return None

        if (dist < minDist):
            minDist = dist
            nearNode = node    
    return nearNode

def steer(near_node, rand_node):
    # unit vector (between two nodes)
    v = rand_node.coord - near_node.coord
    u_v = (v / np.linalg.norm(v))           # unit vector of - v

    # scale it up for the distance (steer distance)
    scaled_coord = u_v * steer_dist                     # this is from the origin like (but we need from near_node point)
    # scaled_coord from near_node
    new_coord = near_node.coord + scaled_coord

    # clamp the new_coord
    new_coord = np.clip(new_coord, 0, 9)

    # return new node
    return Node(new_coord)

def printPath(path):
    for node in path:
        print(f"{node.coord}")

def plotPaths(path, tree):
    # TREE
    coord_x_tree = []
    coord_y_tree = []
    for node in tree:
        coord_x_tree.append(node.coord[0])
        coord_y_tree.append(node.coord[1])

    plt.scatter(coord_x_tree, coord_y_tree, color="red")
    # plt.plot(coord_x_tree, coord_y_tree)

    # plot collision (wall)
    x_box = [2, 4, 4, 2]
    y_box = [0, 0, 6, 6]
    plt.fill(x_box, y_box, color='gray', alpha=1)
    x_box = [6, 8, 8, 6]
    y_box = [3, 3, 9, 9]
    plt.fill(x_box, y_box, color='gray', alpha=1)

    # PATH
    coord_x = []
    coord_y = []
    for node in path:
        coord_x.append(node.coord[0])
        coord_y.append(node.coord[1])

    plt.scatter(coord_x, coord_y)
    plt.plot(coord_x, coord_y)
    plt.show()

def is_collision(new_node):
    x, y = new_node.coord
    # check collision (under the collision area)
    if (x >= C_colli_1[0, 0] and x <= C_colli_1[0, 1]) and (y >= C_colli_1[1, 0] and y <= C_colli_1[1, 1]):
        return True
    if (x >= C_colli_2[0, 0] and x <= C_colli_2[0, 1]) and (y >= C_colli_2[1, 0] and y <= C_colli_2[1, 1]):
        return True
    return False

##


# empty world
C_space = np.zeros((10, 10))        # C_space
start_coord = (0,0)
goal_coord = (9,9)

# initialize collision areas
C_colli_1 = np.array([[2, 4],      # coord_x under collision
                    [0, 6]])      # coord_y under collision
C_colli_2 = np.array([[6, 8],      # coord_x under collision
                    [3, 9]])      # coord_y under collision

K = 1000
steer_dist = 0.5  # for now keeping steering distance as constant
goal_threshold = 0.5  # again for now (it should be lot closer)
reachedGoal = False

rng = np.random.default_rng(seed=42)

# start node
start = Node(start_coord, None)     # start node does not have any parent
# tree
tree = []
tree.append(start)

startTime = time.perf_counter()

for idx in range(K):
    # generate random node in free C-space
    rand_node = Node(get_rand_coord())
    # find the nearest of node to the randomly generated node
    near_node = nearest(tree, rand_node)

    if (near_node is None):
        print("Random node failed (no near node found in the tree)")
        continue

    # steer towards the randomly generated node
    new_node = steer(near_node, rand_node)

    # check collision
    if (is_collision(new_node)):
        print("New node under collision - going back")
        continue

    # update parent for back tracking (path finding)
    new_node.parent = near_node
    tree.append(new_node)

    # check closness of new node to goal node (using thresholding)
    goal_dist = np.linalg.norm(np.array(goal_coord) - new_node.coord)
    if (goal_dist <= goal_threshold):
        # if distance is minimal (connect to the goal node)
        goal_node = Node(goal_coord, new_node)
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

    if (np.all(np.equal(tempNode.parent.coord, np.array(start_coord)))):
        startReached = True

path.reverse()

print("\nComputed Path\n")
printPath(path)

print(f"\nTime Taken: {time.perf_counter() - startTime} sec")

plotPaths(path, tree)