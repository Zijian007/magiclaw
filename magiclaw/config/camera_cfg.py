#!/usr/bin/env python

"""
Camera configuration
===

This module contains the configuration for the camera.
"""

import os
import numpy as np
import yaml
from typing import Optional

class CameraConfig:
    """
    Camera configuration class.
    
    Attributes:
        mode (str): The mode of the camera. Can be "web" or "usb".
        host (str): The host address of the camera.
        port (int): The port number of the camera.
        width (int): The width of the camera image.
        height (int): The height of the camera image.
        dist (np.ndarray): The distortion coefficients of the camera.
        mtx (np.ndarray): The camera matrix.
        marker_size (float): The size of the marker in meters.
        filter_on (bool): Whether to apply a filter to the image.
        filter_frame (int): The size of the filter kernel.
        marker2global_tvec (np.ndarray): The translation vector from the marker to the global frame.
        marker2global_rmat (np.ndarray): The rotation vector from the marker to the global frame.
    """

    def __init__(
        self,
        mode: str = "web",
        host: Optional[str] = None,
        port: int = 5555,
        width: int = 320,
        height: int = 240,
        dist: np.ndarray = np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
        mtx: np.ndarray = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
        marker_size: float = 0.008,
        marker_num: int = 1,
        filter_on: bool = True,
        filter_frame: int = 5,
        marker2global_tvec: np.ndarray = np.array([0.0, 0.0, 0.0]),
        marker2global_rmat: np.ndarray = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
    ) -> None:
        """
        Initialize the camera configuration.
        
        Args:
            mode (str): The mode of the camera. Can be "web" or "usb".
            host (str): The host address of the camera.
            port (int): The port number of the camera.
            width (int): The width of the camera image.
            height (int): The height of the camera image.
            dist (np.ndarray): The distortion coefficients of the camera.
            mtx (np.ndarray): The camera matrix.
            marker_size (float): The size of the marker in meters.
            marker_num (int): The number of markers to detect.
            filter (bool): Whether to apply a filter to the image.
            filter_size (int): The size of the filter kernel.
            marker2global_translation (np.ndarray): The translation vector from the marker to the global frame.
            marker2global_rotation (np.ndarray): The rotation vector from the marker to the global frame.
        """
        
        self.mode = mode
        self.host = host
        self.port = port
        self.width = width
        self.height = height
        self.dist = dist
        self.mtx = mtx
        self.marker_size = marker_size
        self.marker_num = marker_num
        self.filter_on = filter_on
        self.filter_frame = filter_frame
        self.marker2global_tvec = marker2global_tvec
        self.marker2global_rmat = marker2global_rmat
        
    def read_config_file(self, file_path: str, root_dir: str = ".") -> None:
        """
        Read the camera configuration from a yaml file.
        
        Args:
            file_path (str): The path to the yaml configuration file.
            root_dir (str): The root directory to resolve relative paths.
        """
        
        with open(os.path.join(root_dir, file_path), "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            
            for key, value in config.items():
                if hasattr(self, key):
                    setattr(self, key, value)
        
        