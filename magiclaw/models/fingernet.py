#!/usr/bin/env python

"""
FingerNet
=========

FingerNet is a neural network model for infering the proprioception of the soft finger.
The input is the motion, and the output is the force on the bottom surface and the displacement of mesh nodes.
This module is to deploy the model using ONNX runtime.

Usage
-----------------

To use the FingerNet model, you can initialize the FingerNet class by:

```
from magiclaw.models.fingernet import FingerNet

finger_net = FingerNet(
    name="FingerNet",
    model_path=<path_to_your_model>,
    device="auto"
)
```

where `<path_to_your_model>` is the path to your ONNX model file.

To run inference, you can use the `infer` method of the `FingerNet` class.

```
force, node = finger_net.infer(motion)
```

where `motion` (np.ndarray) is the input motion data, and `force` (np.ndarray) and `node` (np.ndarray) are the 
output force on the bottom surface and the node displacement of the finger, respectively.

For more information, please refer to https://github.com/asMagiClaw/metafinger
"""

import argparse
from typing import Tuple
import numpy as np
from magiclaw.utils.nn_utils import init_model


class FingerNet:
    """
    FingerNet class.

    This class is used to initialize the FingerNet model and run inference.
    The model is implemented in PyTorch and exported to ONNX format for inference.

    The model input is the motion of the finger.
    The model output is the force on the bottom surface and the displacement of mesh nodes.
    
    Attributes:
        name (str): The name of the model.
        model_path (str): The path of the model.
        model (onnxruntime.InferenceSession): The ONNX runtime model.
    """

    def __init__(self, name: str, model_path: str, device: str = "auto") -> None:
        """
        FingerNet initialization.

        Args:
            name (str): The name of the model.
            model_path (str): The path of the model.
            device (str): The device to run the model on, selecting "auto" (default), "cpu", "cuda", and "hailo".
        Raises:
            ValueError: If the model path is not valid or the model cannot be loaded.
        """

        # Set the name and model path
        self.name = name
        self.model_path = model_path

        # Create a ONNX runtime model
        try:
            self.model = init_model(self.model_path, device)
        except Exception as e:
            raise ValueError(f"Failed to load the model: {e}")

        # Print the initialization message
        print("{:-^80}".format(f" {self.name} Initialization "))
        print("Model Path:", self.model_path)
        print(
            "Input:",
            [
                f"{input.name} ({input.shape[0]}, {input.shape[1]})"
                for input in self.model.get_inputs()
            ],
        )
        print(
            "Output:",
            [
                f"{output.name} ({output.shape[0]}, {output.shape[1]})"
                for output in self.model.get_outputs()
            ],
        )
        print("Model Initialization Done.")
        print("{:-^80}".format(""))

    def infer(self, motion: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Inference.

        This function runs the model with the given motion input and returns the force and node displacement.

        Args:
            motion (np.ndarray): The motion of the finger.

        Returns:
            force (np.ndarray): The force on the bottom surface of the finger.
            node (np.ndarray): The node displacement of the finger.
        """

        return self.model.run(
            None, {"motion": motion.astype(np.float32).reshape(1, -1)}
        )


if __name__ == "__main__":
    # Parse the arguments
    parser = argparse.ArgumentParser(description="FingerNet inference.")
    parser.add_argument(
        "--name",
        type=str,
        default="FingerNet",
        help="The name of the model.",
    )
    parser.add_argument(
        "--model_path",
        type=str,
        default="./models/FingerNet.onnx",
        help="The path of the model.",
    )
    args = parser.parse_args()

    # Initialize the FingerNet
    finger_net = FingerNet(args.name, args.model_path)
