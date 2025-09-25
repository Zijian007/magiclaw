#!/usr/bin/env python

"""
Utility functions for neural network in MagiClaw.
"""

import os
import onnxruntime as ort


def init_model(model_path: str, device: str = "auto") -> ort.InferenceSession:
    """
    Initialize an ONNX model.

    Args:
        model_path (str): The path to the ONNX model file.
        device (str): The device to be used for inference. Options are "auto", "cuda", "hailo", or "cpu".

    Returns:
        ort.InferenceSession: The loaded ONNX model.
    """

    if not model_path.endswith(".onnx"):
        raise ValueError("The model path must end with .onnx.")
    if not os.path.exists(model_path):
        raise ValueError("The model path does not exist.")

    sess_options = ort.SessionOptions()
    sess_options.intra_op_num_threads = 1
    sess_options.inter_op_num_threads = 1
    sess_options.log_severity_level = 3

    return ort.InferenceSession(
        model_path, sess_options, providers=[get_provider(device)]
    )


def get_provider(devices: str = "auto") -> str:
    """
    Get the device for ONNX model inference.

    This function checks if CUDA, or Hailo are available,
    and returns the appropriate provider for ONNX model inference.
    If none of these providers are available, it defaults to CPUExecutionProvider.

    Returns:
        provider (str): The provider to be used for ONNX model inference.
    Raises:
        ValueError: If the specified device is not supported or available.
    """

    available_providers = ort.get_available_providers()

    if devices == "cuda":
        if "CUDAExecutionProvider" in available_providers:
            return "CUDAExecutionProvider"
        else:
            raise ValueError("CUDAExecutionProvider is not available.")
    elif devices == "hailo":
        if "HailoExecutionProvider" in available_providers:
            return "HailoExecutionProvider"
        else:
            raise ValueError("HailoExecutionProvider is not available.")
    elif devices == "auto" or devices == "cpu":
        return "CPUExecutionProvider"
    else:
        raise ValueError("Unsupported device type.")
