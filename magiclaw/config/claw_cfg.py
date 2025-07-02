#!/usr/bin/env python

"""
Claw configuration
===

This module contains the configuration for the claw.
"""

import os
import yaml
from .motor_cfg import MotorConfig


class ClawConfig:
    """
    Claw configuration class.

    Attributes:
        claw_id (int): The ID of the claw.
        lead (float): The lead of the screw.
        gear_radius (float): The radius of the gear.
        motor_id (int): The ID of the motor.
        bus_interface (str): The bus interface for the motor.
        bus_channel (int): The bus channel for the motor.
        Kp_s (float): Proportional gain for position control.
        Kp_b (float): Proportional gain for velocity control.
        Kd (float): Derivative gain for velocity control.
        iq_max (float): Maximum current for the motor.
        angle_range (tuple): The range of angles for the claw.
    """

    def __init__(
        self,
        claw_id: int = 0,
        lead: float = 15.0,
        gear_radius: float = 15.0,
        motor_config: MotorConfig = MotorConfig(),
    ) -> None:
        """
        Initialize the claw configuration.

        Args:
            claw_id (int): The ID of the claw.
            lead (float): The lead of the screw.
            gear_radius (float): The radius of the gear.
            motor_config (MotorConfig): The motor configuration object.
        """

        self.claw_id = claw_id
        self.lead = lead
        self.gear_radius = gear_radius
        self.motor_id = motor_config.id
        self.bus_interface = motor_config.bus_interface
        self.bus_channel = motor_config.bus_channel
        self.Kp_b = motor_config.Kp_b
        self.Kp_s = motor_config.Kp_s
        self.Kd_b = motor_config.Kd_b
        self.Kd_s = motor_config.Kd_s
        self.iq_max = motor_config.iq_max
        self.angle_range = motor_config.angle_range
        self.motor_angle_deadband = motor_config.angle_deadband
        self.motor_speed_deadband = motor_config.speed_deadband

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
                elif key == "motor":
                    for motor_key, motor_value in value.items():
                        if hasattr(self, motor_key):
                            setattr(self, motor_key, motor_value)

    def set(self, name: str, value) -> None:
        """
        Set an attribute of the motor configuration.

        Args:
            attr_name (str): The name of the attribute to set.
            value: The value to set for the attribute.
        """
        
        if hasattr(self, name):
            setattr(self, name, value)
        else:
            raise AttributeError(f"MotorConfig has no attribute '{name}'")