#!/usr/bin/env python

"""
FingerNet Inference

FingerNet is a neural network model for infering the proprioception of the soft finger.
The input is the motion, and the output is the force on the bottom surface and the displacement of mesh nodes.
The model is implemented in PyTorch and exported to ONNX format for inference.

Example usage:
```bash
python finger_net.py --name FingerNet --model_path ./models/FingerNet.onnx
```

For more information, please refer to https://github.com/asMagiClaw/metafinger
"""

import argparse
import os
from typing import Tuple
import onnxruntime
import numpy as np

class FingerNet:
    def __init__(self, name, model_path) -> None:
        """FingerNet initialization.

        Args:
            name: The name of the model.
            model_path: The path of the model.
        """
        
        # Set the name and model path
        self.name = name
        self.model_path = model_path
        if not self.model_path.endswith(".onnx"):
            raise ValueError("\033[31mThe model path must end with .onnx\033[0m")
        if not os.path.exists(self.model_path):
            raise ValueError("\033[31mThe model path does not exist\033[0m")
        
        # Create a ONNX runtime session
        self.session = onnxruntime.InferenceSession(self.model_path)
        
        # Print the initialization message
        print("{:-^80}".format(f" {self.name} Initialization "))
        print("Model Path:", self.model_path)
        print("Input:", [f"{input.name} ({input.shape[0]}, {input.shape[1]})" for input in self.session.get_inputs()])
        print("Output:", [f"{output.name} ({output.shape[0]}, {output.shape[1]})" for output in self.session.get_outputs()])
        print("Model Initialization Done.")
        print("{:-^80}".format(""))
     
    def infer(self, motion: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Inference.

        Args:
            motion (np.ndarray): The motion of the finger.
            
        Returns:
            force (np.ndarray): The force on the bottom surface of the finger.
            node (np.ndarray): The node displacement of the finger.
        """

        return self.session.run(None, {"motion": motion.astype(np.float32).reshape(1, -1)})
    

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
    
    # Given a random motion and infer the force and node
    print("Given a random motion and infer the force and node...")
    motion = np.concatenate([10*np.random.rand(1, 2), 3*np.random.rand(1, 1), 0.3*np.random.rand(1, 3)], axis=1)
    print("Motion:", motion)
    force, node = finger_net.infer(motion)
    print("Force:", force)
    print("Node:", node)