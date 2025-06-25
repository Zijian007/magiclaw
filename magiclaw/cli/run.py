#!/usr/bin/env python

"""
Run MagiClaw
============

This script is to run the MagiClaw system, providing a command line interface (CLI).

Usage
-----------------

To run MagiClaw, use the following command:

```
run-magiclaw --id <id> --mode <mode> --loop_rate <loop_rate> --phone_host <phone_host> --bilateral_host <bilateral_host>
```

where `<id>` is the MagiClaw's ID (corresponding to the `CAN` interface, default is 0), `<mode>` is the operation mode
(including `standalone` and `teleop`, default is `standalone`), `<loop_rate>` is the loop rate in Hz (default is 30),
`<phone_host>` is the host address for the phone (overwrite the default value in the config file, optional), and
`<bilateral_host>` is the host address for the bilateral communication (only required in `teleop` mode, overwrite the
default value in the config file, optional).

Note that if the CLI is installed under `venv` or `conda`, make sure to setup the environment first, or directly run
the script with the environment directory, e.g., for a `miniconda` environment:

```
/home/user/miniconda3/bin/run-magiclaw
```
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
    parser.add_argument(
        "--phone_host",
        type=str,
        required=False,
        help="The host address for the phone."
    )
    parser.add_argument(
        "--bilateral_host",
        type=str,
        required=False,
        help="The host address for the bilateral communication."
    )
    args = parser.parse_args()

    # Initialize MagiClaw
    magiclaw = MagiClaw(id=args.id, mode=args.mode, loop_rate=args.loop_rate, phone_host=args.phone_host, bilateral_host=args.bilateral_host)

    # Run MagiClaw
    magiclaw.run()
    
if __name__ == "__main__":
    main()