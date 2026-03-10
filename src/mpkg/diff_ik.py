#!/usr/bin/env python3

"""
Differential IK solver implementation for robotic arms.
NOTE: No collision check is performed.
"""


import time
import pinocchio
import numpy as np
from scipy.spatial.transform import Rotation

from mpkg.utils.pin_utils import get_EE_pose, visualizeAndWait


class IK_Solver:
    def __init__(
        self,
        model,
        collision_model,
        visual_model,
        ee_frame_name: str = "tool0",
        maxIterations: int = 1000,
        trans_error_threshold: int = 0.001,
        ori_error_threshold: int = 0.001,
        visualize_final_config: bool = False
    ):
        
        self.model = model
        self.data = self.model.createData()
        self.collision_model = collision_model
        self.visual_model = visual_model
        self.eeFrameId = model.getFrameId(ee_frame_name)
        self.maxIterations = maxIterations
        self.maxTransError = trans_error_threshold
        self.maxOrieError = ori_error_threshold
        self.dt = 0.01

        self.startTime = None
        self.solved_IK = False
        self.visualize_final_q = visualize_final_config


    def solve_ik(self, cur_q: np.ndarray, targetPose: np.ndarray):
        """Function to compute IK for given EE-pose, using numerical method"""

        if (not targetPose.shape == (4,4)):
            print("Target Pose is not 4x4 transformation matrix")
            return None

        cur_q = np.array(cur_q).reshape((6,))

        # reset parameters
        self.startTime = time.perf_counter()
        self.solved_IK = False
        count = 0

        # main loop
        while True:

            # compute current end effector pose (in base or world frame)
            curEEPose = get_EE_pose(self.eeFrameId, cur_q, self.model, self.data)

            # compute pose error
            eeTtarget = np.linalg.pinv(curEEPose) @ targetPose
            error = np.array(pinocchio.log(eeTtarget).vector).reshape((6,))         # inspired from pinocchio IK implementations

            # check error
            if (
                np.linalg.norm(error[0:3], ord=np.inf) < self.maxTransError and 
                np.linalg.norm(error[3:6], ord=np.inf) < self.maxOrieError
            ):
                print("Found target config!")
                self.solved_IK = True
                break
            
            # check count
            if (count >= self.maxIterations):
                print("Max interations reached!")
                self.solved_IK = False
                break

            # compute jac
            jac = pinocchio.computeFrameJacobian(self.model, self.data, cur_q, self.eeFrameId, pinocchio.LOCAL)
            # compute dq from cartesian vel
            dq = np.linalg.pinv(jac) @ error.reshape((6, 1))            # TODO: need to go for DLS method (more numerically stable)
            cur_q += dq.reshape((6,)) * self.dt

        if (not self.solved_IK):
            print("Unable to find target joint configuration")
            return None

        print(f"Computed joint config(q): \n{cur_q}")
        print(f"Time taken: {round(time.perf_counter() - self.startTime, 4)} sec")

        if (self.visualize_final_q):
            visualizeAndWait(self.model, self.collision_model, self.visual_model, cur_q)

        return cur_q
