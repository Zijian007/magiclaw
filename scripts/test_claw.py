#!/usr/bin/env python

"""
Claw connection test
=================

This script is to test the claw connection.
It initializes the claw based on the provided parameter file.
It controls the claw to finish 3 cycles of opening and closing.

Usage:
-----------------

To run the script, use the following command:

```
python test_claw.py --params_path <params_path>
```

where <params_path> is the path to the claw parameters file.
"""

import argparse
import os
import time
import yaml
from magiclaw.devices.claw import Claw

def claw_test(params_path: str):
    """
    Test the claw.
    
    Args:
        params_path (str): The path of the claw parameters.
    """

    # Initialize claw
    with open(params_path, "r") as f:
        claw_params = yaml.load(f.read(), Loader=yaml.Loader)
    
    claw_name = os.path.basename(params_path).split(".")[0]
    
    # Initialize claw
    claw = Claw(
        claw_id=claw_params["id"],
        lead=claw_params["lead"],
        gear_radius=claw_params["gear_radius"],
        **claw_params["motor"],
    )
    
    # Control the claw
    for i in range(3):
        print(f"Cycle {i+1}: Opening claw...")
        claw.open()
        print("Claw opened.")
        
        time.sleep(1)
        
        print("Closing claw...")
        claw.close()
        print("Claw closed.")
        
        time.sleep(1)
    
    print("Claw test completed.")
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test the claw connection.")
    parser.add_argument(
        "--params_path",
        type=str,
        required=True,
        help="Path to the claw parameters file.",
    )
    args = parser.parse_args()
    
    # Test claw
    claw_test(args.params_path)