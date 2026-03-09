#!/usr/bin/env python3

"""
RRT Star implementation for robotic arms, currently the planner accepts n-vector dim start and goal configurations.
The package can also modified for 2D or even higher dim - configurations.
"""

import time
import pinocchio
import numpy as np
from scipy.spatial.transform import Rotation

from mpkg.utils.rrt_utils import (
    Node
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
        ):
        
        self.model = pinModel
        self.data = self.model.createData()
        self.collision_model = pinCollisionModel
        self.collision_data =  self.collision_model.createData()
        self.visual_model = pinVisualModel
        self.targetFrameId = self.model.getFrameId(target_frame_name)
        






if __name__ == "__main__":
    model, collision_model, visual_model = pinocchio.buildModelsFromUrdf("/home/logesh/fanuc_ws/src/fanuc_ros2_drivers/src/fanuc_description/urdf/lrmate200id4s.urdf")
    srdf_model_path = "/home/logesh/fanuc_ws/src/fanuc_ros2_drivers/src/fanuc_description/urdf/lrmate200id4s.srdf"
    planner = RRTStarPlanner(model, collision_model, visual_model, "tool0")