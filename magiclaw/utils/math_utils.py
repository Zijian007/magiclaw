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

def r_from_euler(order: str, angles: np.ndarray) -> np.ndarray:
    """
    Create a rotation matrix from Euler angles.

    Args:
        order (str): The order of the Euler angles (e.g., 'xyz', 'zyx').
        angles (np.ndarray): The Euler angles in radians.
    
    Returns:
        Rotation (Rotation): A scipy Rotation object representing the rotation.
    """
    
    return R.from_euler(order, angles)

def r_from_quat(quat: np.ndarray) -> R:
    """
    Create a rotation object from a quaternion.

    Args:
        quat (np.ndarray): The quaternion in the format [qx, qy, qz, qw].
    
    Returns:
        Rotation (Rotation): A scipy Rotation object representing the rotation.
    """
    
    return R.from_quat(quat)

def r_from_matrix(matrix: np.ndarray) -> R:
    """
    Create a rotation object from a rotation matrix.

    Args:
        matrix (np.ndarray): The rotation matrix.
    
    Returns:
        Rotation (Rotation): A scipy Rotation object representing the rotation.
    """
    
    return R.from_matrix(matrix)

def r_to_euler(r: R, order: str = 'xyz') -> np.ndarray:
    """
    Convert a rotation object to Euler angles.

    Args:
        r (Rotation): A scipy Rotation object.
        order (str): The order of the Euler angles (default is 'xyz').
    
    Returns:
        np.ndarray: The Euler angles in radians.
    """
    
    return r.as_euler(order, degrees=False)

def r_to_quat(r: R) -> np.ndarray:
    """
    Convert a rotation object to a quaternion.

    Args:
        r (Rotation): A scipy Rotation object.
    
    Returns:
        np.ndarray: The quaternion in the format [qx, qy, qz, qw].
    """
    
    return r.as_quat()

def r_to_matrix(r: R) -> np.ndarray:
    """
    Convert a rotation object to a rotation matrix.

    Args:
        r (Rotation): A scipy Rotation object.
    
    Returns:
        np.ndarray: The rotation matrix.
    """
    
    return r.as_matrix()