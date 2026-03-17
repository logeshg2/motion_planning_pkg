#!/usr/bin/env python3

"""
This is an example implementation of RRT Connect for path planning for 2D navigation with collision check.
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
        print(f"{node}")

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
        coord_x.append(node[0])
        coord_y.append(node[1])

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

def isCollision_free(node1, node2, step=0.01):
    # unit vector (between two nodes)
    v = node2.coord - node1.coord
    length = np.linalg.norm(v)
    u_v = (v / length)
    
    steps = int(length / step)

    if (steps < 1):
        return True

    step = length / (steps - 1)

    # check points in the line segment between node1 and node2
    for idx in range(steps):
        point = node1.coord + (u_v * (step * idx))
        if (is_collision(Node(point))):
            return False
    
    return True

def Connect(tree, node):
    goal_reached = False

    # not checking any collision
    near_node = nearest(tree, node)

    while True:
        new_node = steer(near_node, node)
        
        if (new_node is None):
            print("here")
            continue

        if (not isCollision_free(near_node, new_node)):
            # return 'false' if new node leads to collision
            goal_reached = False
            break

        dist = np.linalg.norm(node.coord - new_node.coord)
        # print(dist)
        if (dist < 0.3):
            goal_reached = True
            break

        new_node.parent = near_node
        tree.append(new_node)
        near_node = new_node

    return goal_reached

def getPathFromTree(tree):
    path = []
    startNode = tree[-1]

    while (startNode is not None):
        path.append(startNode.coord)
        startNode = startNode.parent
    
    path.reverse()

    return path

def shortcut(path, num_itr=100):
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
        if (isCollision_free(Node(low_node), Node(high_node), 0.01)):
            path = path[:low_idx+1] + path[high_idx:]
            print("Path shortcut applied!")
    
    return path

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
goal = Node(goal_coord, None)

# tree
tree1 = []
tree2 = []
path = []
tree1.append(start)
tree2.append(goal)

startTime = time.perf_counter()

for idx in range(K):
    if np.random.random() < 0.1:
        rand_node = goal
    else:
        # generate random node in free C-space
        rand_node = Node(get_rand_coord())
    # find the nearest of node to the randomly generated node
    near_node = nearest(tree1, rand_node)

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
    tree1.append(new_node)

    # RRT - Connect steps
    goal_reached = Connect(tree2, new_node)

    if (not goal_reached):
        # swap trees
        tree1, tree2 = tree2, tree1
    else:
        reachedGoal = True
        treeNodes = tree1.copy()
        treeNodes.extend(tree2.copy())

        if (np.all(tree1[0].coord == start_coord)):
            tree1 = getPathFromTree(tree1)  # from start
            tree2 = getPathFromTree(tree2)  # from goal
            tree2.reverse()
            path.extend(tree1)
            path.extend(tree2)
        else:
            tree1 = getPathFromTree(tree1)  # from goal
            tree2 = getPathFromTree(tree2)  # from start
            tree1.reverse()
            path.extend(tree2)
            path.extend(tree1)
        break

if (not reachedGoal):
    print("Unable to reach goal this time!!!")
    exit(0)

# apply shortcut
# path = shortcut(path)

print("\nComputed Path\n")
printPath(path)

print(f"\nTime Taken: {time.perf_counter() - startTime} sec")

plotPaths(path, treeNodes)