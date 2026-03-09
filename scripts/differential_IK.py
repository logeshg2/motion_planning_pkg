#! /usr/bin/env python3


import time 
import pinocchio
import numpy as np
from scipy.spatial.transform import Rotation
from pinocchio.visualize import MeshcatVisualizer


## Helper function

def get_ee_pose(q):
    pinocchio.framesForwardKinematics(model, data, q)
    temp = data.oMf[eeFrameId]
    eepose = np.eye(4)
    eepose[0:3, 3] = temp.translation
    eepose[0:3, 0:3] = temp.rotation
    return eepose

##



model, collision_model, visual_model = pinocchio.buildModelsFromUrdf("/home/logesh/fanuc_ws/src/fanuc_ros2_drivers/src/fanuc_description/urdf/lrmate200id4s.urdf")
srdf_model_path = "/home/logesh/fanuc_ws/src/fanuc_ros2_drivers/src/fanuc_description/urdf/lrmate200id4s.srdf"
data = pinocchio.createDatas(model)[0]
eeFrameId = model.getFrameId("tool0")
qz = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# collision pairs setup
collision_model.addAllCollisionPairs()
pinocchio.removeCollisionPairs(model, collision_model, srdf_model_path)     # remove adjacent joints from collision pairs

# jacobian example
# jac = pinocchio.computeFrameJacobian(model, data, qz, eeFrameId, pinocchio.LOCAL)

# fk example
# pinocchio.framesForwardKinematics(model, data, qz)
# ee_pose = data.oMf[eeFrameId]
# ee_pose.translation[2] -= 0.330                       # this is required if you are working with real robot
# print(ee_pose.translation)
# print(Rotation.from_matrix(ee_pose.rotation).as_euler("xyz", degrees=True))


# visualizer
viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)
viz.loadViewerModel()

# simple diff IK
cur_q = qz
startPose = get_ee_pose(cur_q)
targetPose = np.eye(4)
targetPose[0:3, 3] = [0.4, 0.0, 0.3]
targetPose[0:3, 0:3] = Rotation.from_euler("xyz", [0,180,180], degrees=True).as_matrix()

# show start config
viz.display(cur_q)

startTime = time.perf_counter()
# solve IK
while True:
    # read current cartesian position
    curPose = get_ee_pose(cur_q)    # wTee
    
    # compute jac
    jac = pinocchio.computeFrameJacobian(model, data, cur_q, eeFrameId, pinocchio.LOCAL)

    # compute cartesian error
    eeTtarget = np.linalg.pinv(curPose) @ targetPose    # eeTt
    error = np.array(pinocchio.log(eeTtarget).vector)
    
    if (np.linalg.norm(error, ord=np.inf) < 0.001):
        print("convereged!")
        break

    dq = np.linalg.pinv(jac) @ error
    cur_q += dq * 0.01

# after convergence
print(f"\nfinal config: {cur_q}")
print(f"\ntime taken: {time.perf_counter() - startTime}")
input("\nEnter: ")
viz.display(cur_q)
input("\nEnter to quit! ")