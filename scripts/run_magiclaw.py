#!/usr/bin/env python

import argparse
from magiclaw import MagiClaw

# Parse command line arguments
parser = argparse.ArgumentParser(description="Run MagiClaw")
parser.add_argument(
    "--id",
    type=int,
    default=0,
    help="The ID of the claw (default: 0).",
)
parser.add_argument(
    "--mode",
    type=str,
    choices=["teleoperation", "standalone"],
    default="teleoperation",
    help="Mode to run MagiClaw in.",
)
parser.add_argument(
    "--loop_rate",
    type=int,
    default=30,
    help="The loop rate in Hz (default: 30).",
)
args = parser.parse_args()

# Initialize MagiClaw
magiclaw = MagiClaw(id=args.id, mode=args.mode)

# Run MagiClaw
magiclaw.run(loop_rate=args.loop_rate)