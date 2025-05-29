#!/usr/bin/env python

"""
Motor configuration
===

This module contains the configuration for the motor.
"""

import os
import yaml


class MotorConfig:
    """
    Motor configuration class.
    """

    def __init__(
        self,
        id: int = 1,
        bus_interface: str = "socketcan",
        bus_channel: str = "can0",
        Kp_s: float = 2.0e-5,
        Kp_b: float = 5.0e-4,
        Kd_s: float = 2.0e-4,
        Kd_b: float = 5.0e-4,
        iq_max: float = 10.0,
        angle_range: float = 360.0,
    ) -> None:
        """
        Initialize the motor configuration.

        Args:
            id (int): The ID of the motor.
            bus_interface (str): The bus interface of the motor.
            bus_channel (str): The bus channel of the motor.
            Kp_s (float): The proportional gain for spring control.
            Kp_b (float): The proportional gain for bilateral control.
            Kd (float): The derivative gain for control.
            iq_max (float): The maximum current for the motor.
            angle_range (float): The range of the motor angle.
        """
        self.id = id
        self.bus_interface = bus_interface
        self.bus_channel = bus_channel
        self.Kp_s = Kp_s
        self.Kp_b = Kp_b
        self.Kd_s = Kd_s
        self.Kd_b = Kd_b
        self.iq_max = iq_max
        self.angle_range = angle_range
        
        self.angle_deadband = 10
        self.speed_deadband = 10

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
