#!/usr/bin/env python

"""
Phone configuration
===

This module contains the configuration for the phone.
"""

import os
import yaml


class PhoneConfig:
    """
    Phone configuration class.

    Attributes:
        phone_id (int): The ID of the phone.
        host (str): The host address for the phone.
        port (int): The port number for the phone.
        camera_id (int): The camera ID for the phone.
        resolution (tuple): The resolution of the phone camera.
    """

    def __init__(
        self,
        phone_id: int = 0,
        host: str = "localhost",
        port: int = 8000,
        color_width: int = 640,
        color_height: int = 480,
        depth_width: int = 256,
        depth_height: int = 192,
        init_pose: list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ) -> None:
        """
        Initialize the phone configuration.

        Args:
            phone_id (int): The ID of the phone.
            host (str): The host address for the phone.
            port (int): The port number for the phone.
            color_width (int): The width of the color camera.
            color_height (int): The height of the color camera.
            depth_width (int): The width of the depth camera.
            depth_height (int): The height of the depth camera.
            init_pose (list): The initial pose of the phone in the format [x, y, z, qx, qy, qz, qw].
        """
        self.phone_id = phone_id
        self.host = host
        self.port = port
        self.color_width = color_width
        self.color_height = color_height
        self.depth_width = depth_width
        self.depth_height = depth_height
        self.init_pose = init_pose

    def read_config_file(self, file_path: str, root_dir: str = ".") -> None:
        """
        Read the phone configuration from a YAML file.

        Args:
            config_path (str): The path to the configuration file.
        """
        
        with open(os.path.join(root_dir, file_path), "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            
            for key, value in config.items():
                if hasattr(self, key):
                    setattr(self, key, value)
                elif isinstance(value, dict):
                    for sub_key, sub_value in value.items():
                        if hasattr(self, f"{key}_{sub_key}"):
                            setattr(self, f"{key}_{sub_key}", sub_value)
                            
    def set_host(self, host: str) -> None:
        """
        Set the host address for the phone.

        Args:
            host (str): The host address.
        """
        self.host = host