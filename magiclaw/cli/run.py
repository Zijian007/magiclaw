#!/usr/bin/env python

"""
Run MagiClaw
============

This script is to run the MagiClaw system.
"""

import argparse
from magiclaw import MagiClaw

def main():
    """
    Main function to run MagiClaw.
    """

    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Run MagiClaw")
    parser.add_argument(
        "--id",
        type=int,
        choices=[0, 1],
        default=0,
        help="The ID of the claw (default: 0).",
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["teleop", "standalone"],
        default="standalone",
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
    magiclaw = MagiClaw(id=args.id, mode=args.mode, loop_rate=args.loop_rate)

    # Run MagiClaw
    magiclaw.run()
    
if __name__ == "__main__":
    main()