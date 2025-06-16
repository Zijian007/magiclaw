#!/usr/bin/env python

"""
Test FingerNet
=================

This script is to test the FingerNet model.
It initializes the FingerNet based on the provided parameter file.
It tests whether the model can be loaded and run successfully.

Usage:
-----------------

To run the script, use the following command:

```bash
python test_fingernet.py --params_path <params_path>
```

where <params_path> is the path to the FingerNet parameters file.
"""

import argparse
import numpy as np
from magiclaw.models.fingernet import FingerNet
from magiclaw.config import FingerNetConfig


def fingernet_test(onnx_path: str):
    """
    Test the FingerNet model.

    Args:
        onnx_path (str): The path of the FingerNet ONNX model file.
    """

    # Initialize FingerNet
    fingernet_cfg = FingerNetConfig(model_path=onnx_path)
    fingernet = FingerNet(fingernet_cfg)

    # Test the model
    print("Give a sample input to the model and check the output.")
    # Generate input
    inputs = np.zeros(
        (1, fingernet.model.get_inputs()[0].shape[1]),
        dtype=np.float32,
    )
    # Infer the model
    outputs = fingernet.infer(inputs)
    # Print the input and output
    print("Input:", inputs)
    for i, output in enumerate(outputs):
        print(f"Output {i}:", output)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test the FingerNet model.")
    parser.add_argument(
        "--onnx_path",
        type=str,
        required=True,
        help="Path to the FingerNet ONNX model file.",
    )
    args = parser.parse_args()

    # Run the test
    fingernet_test(args.onnx_path)
