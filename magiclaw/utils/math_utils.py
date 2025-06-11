#! /usr/bin/env python3

"""
Utility functions for mathematical operations in MagiClaw.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

def convert_pose(pose: np.ndarray, conversion_matrix: np.ndarray) -> np.ndarray:
    """
    Convert a pose by applying a conversion matrix.

    Args:
        pose (np.ndarray): The pose to convert, expected to be a 7-element array (x, y, z, qx, qy, qz, qw).
        conversion_matrix (np.ndarray): The conversion matrix to apply.
    
    Returns:
        np.ndarray: The converted pose, which is also a 7-element array (x, y, z, qx, qy, qz, qw).
    """
    
    mat = np.eye(4, dtype=np.float32)
    mat[:3, :3] = R.from_quat(pose[3:]).as_matrix()
    mat[:3, 3] = pose[:3]
    
    converted_mat = conversion_matrix @ mat
    converted_pose = np.zeros_like(pose, dtype=np.float32)
    converted_pose[:3] = converted_mat[:3, 3]
    converted_pose[3:] = R.from_matrix(converted_mat[:3, :3]).as_quat()
    return converted_pose