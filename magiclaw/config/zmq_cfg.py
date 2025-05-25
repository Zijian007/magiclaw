#!/usr/bin/env python

"""
ZMQ configuration
===

This module contains the configuration for the ZMQ publisher and subscriber.
"""

import yaml


class ZMQConfig:
    """
    ZMQ configuration class.
    """

    def __init__(
        self,
        claw_id: int = 0,
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

        self.phone_host = None
        self.phone_port = 8000

        self.bilateral_host = None

    def read_config_file(self, file_path: str) -> None:
        """
        Read the camera configuration from a yaml file.

        Args:
            file_path (str): The path to the yaml configuration file.
        """

        with open(file_path, "r") as file:
            config = yaml.load(file, Loader=yaml.FullLoader)

            for key, value in config.items():
                if hasattr(self, key):
                    setattr(self, key, value)

    def set_bilateral_host(self, host: str) -> None:
        """
        Set the bilateral host.

        Args:
            host (str): The host address of the bilateral connection.
        """
        self.bilateral_host = host
        
    def set_phone_host(self, host: str) -> None:
        """
        Set the phone host.

        Args:
            host (str): The host address of the phone.
        """
        self.phone_host = host