#!/usr/bin/env python3

"""Utils related fundamental transforms and others"""


import numpy as np
from scipy.spatial.transform import Rotation


def createPoseTransform(
        translation: np.ndarray,
        orientation: np.ndarray,
        orient_type: str = "euler",         # ["euler", "quat", "rotvec"] 
        degrees: bool = True,
        euler_order: str = "xyz",
        scalar_first: bool = False
    ):
    """Function to create 4x4 transformation matrix"""

    assert (orient_type in ["euler", "quat", "rotvec"]), "Wrong orientation type - ['euler', 'quat', 'rotvec']"

    T = np.eye(4)
    T[0:3, 3] = translation

    orientation = np.array(orientation)

    if (orient_type == "euler"):
        rot_obj = Rotation.from_euler(seq=euler_order, angles=orientation.reshape((3,)), degrees=degrees)
        T[0:3, 0:3] = rot_obj.as_matrix()
    
    elif (orient_type == "quat"):
        if (scalar_first):
            temp = orientation.flatten().tolist()
            orientation = temp[1:4] + temp[0:1]

        rot_obj = Rotation.from_quat(quat=orientation.reshape((4,)))
        T[0:3, 0:3] = rot_obj.as_matrix()
    
    elif (orient_type == "rotvec"):
        rot_obj = Rotation.from_rotvec(quat=orientation.reshape((3,)))
        T[0:3, 0:3] = rot_obj.as_matrix()

    return T

