#!/usr/bin/env python

"""
FingerNet configuration
===

This module contains the configuration for the FingerNet model.
"""

from typing import Optional

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
        name: str = "FingerNet",
        model_path: Optional[str] = None,
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