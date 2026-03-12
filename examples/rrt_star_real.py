#!/usr/bin/env python3

"""
This is an example script to control real robotic arm using MPKG motion planning package.
More specifically fanuc robotic arm will be tested.
"""


import time
import pinocchio
import numpy as np

from ComDependencies.robot_controller import robot

from mpkg.diff_ik import IK_Solver
from mpkg.rrt.rrt_star import RRTStarPlanner
from mpkg.utils.base_utils import createPoseTransform
from mpkg.utils.pin_utils import loadModelFromURDF, add_self_collision, getGeomObject, add_object_collision


URDF_PATH = "/home/logesh/fanuc_ws/src/fanuc_ros2_drivers/src/fanuc_description/urdf/lrmate200id4s.urdf"
SRDF_PATH = "/home/logesh/fanuc_ws/src/fanuc_ros2_drivers/src/fanuc_description/urdf/lrmate200id4s.srdf"

bot = robot("192.168.1.9")

def main():
    # read current position
    cur_q = bot.read_current_joint_position()
    cur_q = np.deg2rad(cur_q)
    # remove coupling
    cur_q = bot.remove_joint_coupling(cur_q)

    # load models
    model, collision_model, visual_model = loadModelFromURDF(URDF_PATH)

    ### Compute Target Configuratoin ###
    # numerical IK solver
    ikSolver = IK_Solver(model, collision_model, visual_model)

    # targetPose
    tarpose = createPoseTransform([0.29, 0.3, 0.34], [-180, 0, 0])
    tarpose = createPoseTransform([0.4, 0.0, 0.4], [-180, 0, 0])
    
    # compte IK
    goal_q = ikSolver.solve_ik(cur_q.copy(), tarpose)
    if (goal_q is None):
        exit(0)
    ###


    ### Compute Path for Goal Config ###
    start_q = cur_q
    # rrt star planner
    planner = RRTStarPlanner(
        model, 
        collision_model, 
        visual_model,
        "tool0",
        rng_seed=42,
        visualize=True,
        verbose=False
    )

    # self collision
    add_self_collision(model, collision_model, SRDF_PATH)

    # object collision
    cube1 = getGeomObject("cube1", translation=[0.29, 0.1, 0.4], dimension=[0.1, 0.1, 0.1])
    cube1_id = add_object_collision(cube1, collision_model, visual_model)

    # compute path
    config_path = planner.plan(start_config = start_q, goal_config = goal_q)

    if (config_path is not None):
        print(f"Path length:", len(config_path))
    
    input("Enter to run(real):")

    for config in config_path:
        # add back coupling before sending to robot
        couplled_q = bot.add_joint_coupling(config)
        # rad to deg
        couplled_q = np.rad2deg(couplled_q)

        bot.write_joint_pose(couplled_q, blocking=False)
        
        # wait until arm reaches the target joint config
        error = np.max(np.abs(np.array(couplled_q) - np.array(bot.read_current_joint_position())))
        while (error > 1):
            error = np.max(np.abs(np.array(couplled_q) - np.array(bot.read_current_joint_position())))
            bot.write_joint_pose(couplled_q, blocking=False)
            time.sleep(0.0001)
        
        print("Moving real arm!")
    
    print("Real Arm movement done!")

    ###


if __name__ == "__main__":
    main()