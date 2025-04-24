# !/usr/bin/env python3

"""
Claw Calibration Script

This script is to calibrate the angle range of the claw.
First, the claw closes to the limit and the maximum angle is recorded.
Then, the claw opens to the limit and the minimum angle is recorded.
Therefore, the angle range is calculated, which will be updated to the yaml file in `configs/claw/`.
During deployment, the claw converts the minmimum angle as 0, and the maximum angle as `angle_range`.
"""

import argparse
import os
import time
import yaml
from magiclaw.devices.claw import Claw

# Parse command line arguments
parser = argparse.ArgumentParser(description="Calibrate the claw.")
parser.add_argument(
    "--id",
    type=int,
    default=0,
    help="ID of the claw to calibrate (default: 0)",
)
args = parser.parse_args()

# Check if the configuration file exists
if not os.path.exists(f"configs/claw/claw_{args.id}.yaml"):
    raise FileNotFoundError(
        f"Configuration file for claw {args.id} not found. Please create it first."
    )
# Load the configuration file
with open(f"configs/claw/claw_{args.id}.yaml", "r") as f:
    claw_params = yaml.load(f.read(), Loader=yaml.Loader)

# Close and open the claw to find the angle range for 5 times and calculate the average
print("The claw will close and open to find the angle range for 5 times.")
print("Please make sure the claw is not blocked and connected to the RPi5.")
input("Press `enter` to start...")

# Create a Claw object
claw = Claw(
    claw_id=claw_params["id"],
    lead=claw_params["lead"],
    gear_radius=claw_params["gear_radius"],
    **claw_params["motor"],
)

# Initialize the angle range
angle_max = 0
angle_min = 0

# Loop to close and open the claw
for i in range(5):
    print(f"Round {i + 1}/5")
    
    # Close the claw
    print("Closing the claw...")
    claw.close()
    time.sleep(1)
    
    # Read the motor status
    claw.read_motor_status()
    # Record the maximum angle
    angle_max += claw.motor_angle
    # Print the motor angle
    print(f"Motor angle: {claw.motor_angle:.2f} deg")
    
    # Open the claw
    print("Opening the claw...")
    claw.open()
    time.sleep(1)
    
    # Read the motor status
    claw.read_motor_status()
    # Record the minimum angle
    angle_min += claw.motor_angle
    # Print the motor angle
    print(f"Motor angle: {claw.motor_angle:.2f} deg")

# Calculate the average angle
angle_max = angle_max / 5
angle_min = angle_min / 5
# Calculate the angle range
angle_range = angle_max - angle_min

# Print the angle range
print(f"Angle range: {angle_range:.2f} degrees")

# Update the configuration file
claw_params["motor"]["angle_range"] = round(angle_range, 2)

# Save the updated configuration file
with open(f"configs/claw/claw_{args.id}.yaml", "w") as f:
    yaml.dump(claw_params, f)
print("The configuration file has been updated.")
