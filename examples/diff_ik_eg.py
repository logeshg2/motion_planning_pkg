#!/usr/bin/env python3

"""Example usage of differential ik."""


import numpy as np
from mpkg.diff_ik import IK_Solver
from scipy.spatial.transform import Rotation
from mpkg.utils.pin_utils import loadModelFromURDF


URDF_PATH = "/home/logesh/fanuc_ws/src/fanuc_ros2_drivers/src/fanuc_description/urdf/lrmate200id4s.urdf"
SRDF_PATH = "/home/logesh/fanuc_ws/src/fanuc_ros2_drivers/src/fanuc_description/urdf/lrmate200id4s.srdf"

def main():
    # load models
    model, collision_model, visual_model = loadModelFromURDF(URDF_PATH)

    # numerical IK solver
    ikSolver = IK_Solver(model, collision_model, visual_model, visualize_final_config=True)

    # current_config
    cur_q = np.zeros((6,))
    # targetPose
    tarpose = np.eye(4)
    tarpose[0:3, 3] = [0.4, 0.0, 0.3]
    tarpose[0:3, 0:3] = Rotation.from_euler("xyz", [0, 180, 180], degrees=True).as_matrix()

    # compte IK
    ikSolver.solve_ik(cur_q, tarpose)


if __name__ == "__main__":
    main()