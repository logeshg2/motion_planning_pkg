#!/usr/bin/env python3

"""Example script - motion plan robotic arm in a Mujoco Env"""


import time
import mujoco
import numpy as np
from mujoco.viewer import launch_passive
from scipy.spatial.transform import Rotation

from mpkg.diff_ik import IK_Solver
from mpkg.rrt.rrt_star import RRTStarPlanner
from mpkg.utils.pin_utils import (
    loadModelFromMJCF,
    add_self_collision, 
    getGeomObject, 
    add_object_collision
)
from mpkg.utils.base_utils import createPoseTransform


UR5_XML_PATH = "/home/logesh/mujoco_ws/mujoco_menagerie/universal_robots_ur5e/ur5e.xml"
SCENE_PATH = "/home/logesh/mujoco_ws/mujoco_menagerie/universal_robots_ur5e/scene.xml"
UR5_SRDF_PATH = "/home/logesh/fanuc_ws/src/ur5e.srdf"


### mujoco setup

mjModel = mujoco.MjModel.from_xml_path(SCENE_PATH)
mjData = mujoco.MjData(mjModel)

dt = 0.2
mjModel.opt.timestep = dt

# get all actuator name (no gripper the choosen ur5 robotic arm)
act_names = []
for id in range(mjModel.nu):
    act_names.append(str(mjModel.actuator(id).name))
home_qpos = mjModel.key("home").qpos
home_key_id = mjModel.key("home").id

###

### Pinocchio model

model, collision_model, visual_model = loadModelFromMJCF(UR5_XML_PATH)
add_self_collision(model, collision_model, UR5_SRDF_PATH)


###


### IK Solver

ik_solver = IK_Solver(model, collision_model, visual_model, "attachment_site", visualize_final_config=False)
targetPose = createPoseTransform([0.3, 0.4, 0.3], [-180, 0, 0])
des_q = ik_solver.solve_ik(np.zeros((6,)), targetPose)

if (des_q is None):
    print("Cannot find solution!")
    exit(0)

###


### RRT* planner

# add objects to pin env
cube_geom = getGeomObject("cube1", translation=[0.4, 0.0, 0.3], dimension=[0.3, 0.3, 0.3])
cube_id_1 = add_object_collision(cube_geom, collision_model, visual_model)

# rrt star planner
planner = RRTStarPlanner(
    model, 
    collision_model, 
    visual_model,
    "attachment_site",
    # rng_seed=42,
    visualize=False,
    verbose=False
)
start_q = home_qpos
goal_q = des_q

# compute path
config_path = planner.plan(start_config = start_q, goal_config = goal_q)

if (config_path is None):
    print("Cannot find path!")
    exit(0)

cur_idx = 0

###


with launch_passive(mjModel, mjData, show_left_ui=False, show_right_ui=False) as viewer:

    mujoco.mj_resetDataKeyframe(mjModel, mjData, home_key_id)
    mujoco.mjv_defaultFreeCamera(mjModel, viewer.cam)

    # this will show up frames
    # viewer.opt.frame = 1
    # viewer.opt.sitegroup[:] = 1

    while viewer.is_running():
        start_time = time.time()

        if (cur_idx >= len(config_path)):
            print("Goal reached!")
            time.sleep(3)
            viewer.close()
        else:
            mjData.ctrl[:] = config_path[cur_idx]

            if (np.linalg.norm(mjData.qpos - config_path[cur_idx], ord=np.inf) < 0.01):
                print(f"position-{cur_idx} completed")
                cur_idx += 1

        # adding custom collision object
        # reset user scene
        viewer.user_scn.ngeom = 0
        geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
        viewer.user_scn.ngeom += 1
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_BOX,
            np.array([0.15, 0.15, 0.15]),
            np.array([-0.4, 0.0, 0.3]),
            np.eye(3).flatten(),
            np.array([1,0,0,1])
        )

        mujoco.mj_step(mjModel, mjData)
        viewer.sync()

        remaining_time = time.time() - start_time
        if (remaining_time > 0):
            time.sleep(remaining_time)


def main():
    pass



if __name__ == "__main__":
    main()