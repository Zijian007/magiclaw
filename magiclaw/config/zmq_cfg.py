#!/usr/bin/env python

"""
ZMQ configuration
===

This module contains the configuration for the ZMQ publisher and subscriber.
"""

import os
import yaml
from typing import Optional


class ZMQConfig:
    """
    ZMQ configuration class.

    Attributes:
        claw_id (int): The ID of the claw.
        public_host (str): The host address for the ZMQ connection.
        claw_port (int): The port for the claw connection.
        finger_0_port (int): The port for finger 0 connection.
        finger_1_port (int): The port for finger 1 connection.
        publish_port (int): The port for publishing data.
        phone_host (str): The host address of the phone.
        phone_port (int): The port for the phone connection.
        bilateral_host (str): The host address of the bilateral connection.
    """

    def __init__(
        self,
        claw_id: int = 0,
        bilateral_host: Optional[str] = None,
    ) -> None:
        """
        Initialize the ZMQ configuration.

        Args:
            claw_id (int): The ID of the claw.
            phone_host (str): The host address of the phone.
            bilateral_host (str): The host address of the bilateral connection.
        """
        self.claw_id = claw_id

        self.public_host = "0.0.0.0"

        if self.claw_id == 0:
            self.claw_port = 5301
            self.finger_0_port = 5302
            self.finger_1_port = 5303
            self.publish_port = 6300
        elif self.claw_id == 1:
            self.claw_port = 5401
            self.finger_0_port = 5402
            self.finger_1_port = 5403
            self.publish_port = 6400
        else:
            raise ValueError("Invalid claw ID. Must be 0 or 1.")

        self.bilateral_host = bilateral_host

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

    def set_host(self, host: str) -> None:
        """
        Set the host address for the phone.

        Args:
            host (str): The host address.
        """
        self.host = host
    
    def set_bilateral_host(self, host: str) -> None:
        """
        Set the bilateral host.

        Args:
            host (str): The host address of the bilateral connection.
        """
        
        self.bilateral_host = host
