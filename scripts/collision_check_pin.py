#!/home/logesh/mujoco_ws/mujoco_env/bin/python3

"""
This is an example script for debug purpose - using Pinocchio for both self + external collision checking
"""

import coal
import pinocchio
import numpy as np
from pinocchio.visualize import MeshcatVisualizer

model, collision_model, visual_model = pinocchio.buildModelsFromUrdf("/home/logesh/fanuc_ws/src/fanuc_ros2_drivers/src/fanuc_description/urdf/lrmate200id4s.urdf")
data = pinocchio.createDatas(model)[0]

srdf_model_path = "/home/logesh/fanuc_ws/src/fanuc_ros2_drivers/src/fanuc_description/urdf/lrmate200id4s.srdf"

# print(pinocchio.randomConfiguration(model))
qz = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(6,1)

eeFrameId = model.getFrameId("tool0")

### EE jacobian example
"""
# end effector - jacobian
jac = pinocchio.computeFrameJacobian(model, data, qz, eeFrameId)
print(np.round(jac, 4))

tempVel = np.array([0, 0, 1, 0, 0, 0]).reshape((6,1))
print(np.round(np.linalg.pinv(jac) @ tempVel, 4))
"""

### visualization example
"""
viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)

viz.loadViewerModel()

q0 = pinocchio.neutral(model)
q_rand = pinocchio.randomConfiguration(model)
viz.display(qz)
viz.displayVisuals(True)

while True:
    pass
"""

# collision avoidance
collision_model.addAllCollisionPairs()
pinocchio.removeCollisionPairs(model, collision_model, srdf_model_path)     # remove adjacent joints from collision pairs

# custom object
cube_geom = pinocchio.GeometryObject(
    "cube",
    0,                 # parent joint (0 = world)
    pinocchio.SE3(np.eye(3), np.array([0.0, 0.0, 0.3])),
    coal.Box(1, 1, 1)
)
cube_geom.meshColor = np.array([0.5, 0.5, 0.5, 0.5])
cube_id = collision_model.addGeometryObject(cube_geom)
visual_model.addGeometryObject(cube_geom)

for i in range(cube_id):
    collision_model.addCollisionPair(
        pinocchio.CollisionPair(i, cube_id)
    )

geom_data = pinocchio.GeometryData(collision_model)
q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q1 = np.array([0.0, 0.467, -1.204, 0.0, -2.094, 0.0])

pinocchio.computeCollisions(model, data, collision_model, geom_data, q, False)

# Print the status of collision for all collision pairs
for k in range(len(collision_model.collisionPairs)):
    cr = geom_data.collisionResults[k]
    cp = collision_model.collisionPairs[k]
    print(
        "collision pair:",
        cp.first,
        ",",
        cp.second,
        "- collision:",
        "Yes" if cr.isCollision() else "No",
    )
    name1 = collision_model.geometryObjects[cp.first].name
    name2 = collision_model.geometryObjects[cp.second].name
    print(f"Name of paris: {name1} - {name2}\n")

# visualize the arm position
viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)

viz.loadViewerModel()

viz.display(q)
viz.displayVisuals(True)
viz.displayCollisions(True)

input()