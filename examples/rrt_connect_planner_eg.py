#!/usr/bin/env python3

"""Example usage of RRT Connect Planner. - without object collision"""


import numpy as np

from mpkg.diff_ik import IK_Solver
from mpkg.rrt.rrt_connect import RRTConnectPlanner
from mpkg.utils.base_utils import createPoseTransform
from mpkg.utils.pin_utils import loadModelFromURDF, add_self_collision, visualizeAndWait


URDF_PATH = "/home/logesh/fanuc_ws/src/fanuc_ros2_drivers/src/fanuc_description/urdf/lrmate200id4s.urdf"
SRDF_PATH = "/home/logesh/fanuc_ws/src/fanuc_ros2_drivers/src/fanuc_description/urdf/lrmate200id4s.srdf"

def main():
    # load models
    model, collision_model, visual_model = loadModelFromURDF(URDF_PATH)

    ### Compute Target Configuratoin ###
    # numerical IK solver
    ikSolver = IK_Solver(model, collision_model, visual_model)

    # current_config
    cur_q = np.zeros((6,))
    # targetPose
    tarpose = createPoseTransform([0.3, -0.3, 0.1], [0, 180, 180])
    
    # compte IK
    goal_q = ikSolver.solve_ik(cur_q.copy(), tarpose)
    if (goal_q is None):
        exit(0)
    ###

    ### Compute Path for Goal Config ###
    start_q = cur_q
    # rrt star planner
    planner = RRTConnectPlanner(
        model, 
        collision_model, 
        visual_model,
        "tool0",
        rng_seed=42,
        visualize=True,
        verbose=True
    )

    # self collision
    add_self_collision(model, collision_model, SRDF_PATH)

    # compute path
    config_path = planner.plan(start_config = start_q, goal_config = goal_q)

    if (config_path is not None):
        print(f"Path length:", len(config_path))
    ###



if __name__ == "__main__":
    main()