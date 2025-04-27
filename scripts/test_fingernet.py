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
import os
import yaml
import numpy as np
from magiclaw.models.fingernet import FingerNet


def fingernet_test(params_path: str):
    """
    Test the FingerNet model.
    
    Args:
        params_path (str): The path of the FingerNet parameters.
    """

    # Initialize FingerNet
    with open(params_path, "r") as f:
        fingernet_params = yaml.load(f.read(), Loader=yaml.Loader)
    fingernet_name = os.path.basename(params_path).split(".")[0]
    
    # Initialize FingerNet
    fingernet = FingerNet(
        name=fingernet_name,
        model_path=fingernet_params["model_path"],
        device=fingernet_params["device"],
    )
    
    # Test the model
    print("Give a sample input to the model and check the output.")
    # Generate input
    inputs = np.zeros(
        (1, fingernet.model.get_inputs()[0].shape[1]),
        dtype=np.float32,
    )
    # Infer the model
    outputs = fingernet.infer(input)
    # Print the input and output
    print("Input:", inputs)
    for i, output in enumerate(outputs):
        print(f"Output {i}:", output)
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test the FingerNet model.")
    parser.add_argument(
        "--params_path",
        type=str,
        default="./configs/fingernet/fingernet.yaml",
        help="The path to the FingerNet parameters file.",
    )
    args = parser.parse_args()

    # Run the test
    fingernet_test(args.params_path)