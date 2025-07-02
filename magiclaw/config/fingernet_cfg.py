#!/usr/bin/env python

"""
FingerNet configuration
===

This module contains the configuration for the FingerNet model.
"""


class FingerNetConfig:
    """
    FingerNet configuration class.

    Attributes:
        name (str): The name of the model.
        model_path (str): The path of the model.
        device (str): The device to use for inference. Default is "auto".
    """

    def __init__(
        self,
        model_path: str,
        name: str = "FingerNet",
        device: str = "auto",
    ) -> None:
        """
        Initialize the FingerNet configuration.

        Args:
            name (str): The name of the model.
            model_path (str): The path of the model.
            device (str): The device to use for inference. Default is "auto".
        """

        if model_path is None:
            raise ValueError("model_path must be provided.")

        self.name = name
        self.model_path = model_path
        self.device = device
        
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
