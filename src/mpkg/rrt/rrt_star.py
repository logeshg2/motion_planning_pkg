#!/usr/bin/env python3

"""
RRT Star implementation for robotic arms, currently the planner accepts n-vector dim start and goal configurations.
The package can also modified for 2D or even higher dim - configurations.
"""

import time
import pinocchio
import numpy as np
from scipy.spatial.transform import Rotation
from pinocchio.visualize import MeshcatVisualizer

from mpkg.utils.rrt_utils import (
    Node,
    steer,
    nearest,
    upDateTree,
    shortcut,
    discretize_joint_position,
    getPathNodesFromTree,
    printPath
)

from mpkg.utils.pin_utils import (
    get_random_config,
    isInCollision,
    isCollision_free
)


class RRTStarPlanner:
    def __init__(
            self,
            pinModel,
            pinCollisionModel,
            pinVisualModel,
            target_frame_name: str,
            rng_seed: int = None,
            max_iterations: int = 2000,
            neighbour_radius: int = 0.2,
            steer_distance: int = 0.3,
            goal_threshold: int = 0.5,
            goal_bias: float = 0.20,
            do_shortcutting:bool = True,
            visualize:bool = False
        ):
        
        self.model = pinModel
        self.collision_model = pinCollisionModel
        self.visual_model = pinVisualModel
        self.targetFrameId = self.model.getFrameId(target_frame_name)
        self.jointLowerLimits = self.model.lowerPositionLimit
        self.jointUpperLimits = self.model.upperPositionLimit
        
        # RRT* (planning parameters)
        self.maxIterations = max_iterations
        self.rng_seed = rng_seed
        self.neigh_radius = neighbour_radius
        self.steer_dist = steer_distance
        self.goal_threshold = goal_threshold
        self.goal_bias = goal_bias
        self.reachedGoal = False
        self.tree = []
        self.startTime = None
        self.do_shortcutting = do_shortcutting
        self.visualize = visualize

        # set seed (rng)
        if (self.rng_seed is not None):
            pinocchio.seed(self.rng_seed)     # int(time.time())
            # np.random.seed(self.seed_num)


    def plan(self, start_config: list, goal_config: list):
        """Function perform RRT* to compute path between start config and goal config"""

        if (isInCollision(self.model, self.collision_model, start_config)):
            print("Start Configuration is in collision!")
            return None
        
        if (isInCollision(self.model, self.collision_model, goal_config)):
            print("Goal Configuration is in collision!")
            return None

        # reset paramters
        self.tree = []
        self.reachedGoal = False
        self.startTime = time.perf_counter()

        self.tree.append(Node(start_config))

        # planning loop
        for itr in range(self.maxIterations):
            
            # generate random config
            # NOTE: goal bias is IMP for exploitation
            # NOTE: random config is IMP for exploration
            if (np.random.random() < self.goal_bias):
                rand_node = Node(goal_config)
            else:
                rand_node = Node(get_random_config(self.model))
            
            # nearest node to random node
            near_node = nearest(self.tree, rand_node)

            if (near_node is None):
                print("Random node failed (no near node found in the tree)")
                continue
            
            # steer towards the randomly generated node
            new_node = steer(near_node, rand_node, self.steer_dist, self.jointLowerLimits, self.jointUpperLimits)

            # check collision
            if (not isCollision_free(near_node, new_node, self.model, self.collision_model)):
                print("New node under collision - going back")
                continue

            # update parent for back tracking (path finding)
            new_node.parent = near_node
            self.tree.append(new_node)

            # RRT* additional steps
            upDateTree(self.tree, self.neigh_radius, self.model, self.collision_model)

            # check closness of new node to goal node (using thresholding)
            goal_dist = np.linalg.norm(np.array(goal_config) - new_node.q)

            if (goal_dist <= self.goal_threshold):
                # if distance is minimal (connect to the goal node)
                goal_cost = new_node.cost + goal_dist
                goal_node = Node(goal_config, new_node, goal_cost)
                self.tree.append(goal_node)

                self.reachedGoal = True
                print("Goal Position reached!")
                break
        
        if (not self.reachedGoal):
            print("Unable to reach goal this time!!!")
            return None
        
        # get path from tree
        path_nodes = getPathNodesFromTree(self.tree)
        
        # perform shortcutting
        if (self.do_shortcutting):
            path_nodes = shortcut(path_nodes, self.model, self.collision_model, num_itr=100)      
        print("here")
        # discretize the path
        discretized_path = discretize_joint_position(path_nodes)

        # logging
        print("\nComputed Path\n")
        printPath(path_nodes)

        print(f"\nTotal Cost: {path_nodes[-1].cost}")
        print(f"\nTime taken: {time.perf_counter() - self.startTime} sec\n")

        if (self.visualize):
            self.visualizePath(discretized_path)

        return discretized_path
    

    def visualizePath(self, path):
        """Function to visualize the computed path using meshcat"""

        print("Visualizing the computed path")
        viz = MeshcatVisualizer(self.model, self.collision_model, self.visual_model)
        viz.initViewer(open=True)
        viz.loadViewerModel()

        viz.display(path[0])
        time.sleep(0.05)

        input("Enter to simulate:")
        for q_p in path:
            viz.display(q_p)
            time.sleep(0.01)
        input("Enter to quit:")