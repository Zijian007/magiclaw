#!/usr/bin/env python

"""
MagiClaw
=========

MagiClaw is a Python package for controlling the MagiClaw, an intelligent data collection system.
The MagiClaw consists of a claw, two fingers, and a phone.
It provides a set of classes and functions to initialize and run all components.

Before running this module, make sure to set up all necessary configurations in `./configs/`.

Usage
-----------------

To run Magiclaw, use the following command:

```
from magiclaw import MagiClaw

magiclaw = MagiClaw(id=<id>, mode=<mode>)
magiclaw.run(loop_rate=<loop_rate>)
```

where `<id> (default: 0)` is the ID of the claw, <mode> (default: "teleoperation") is the mode to run MagiClaw in,
and <loop_rate> (default: 30) is the loop rate in Hz.
"""

import argparse
import time
import yaml
from magiclaw.modes import teleoperation_processes, standalone_processes
from magiclaw.utils.logging_utils import init_logger


class MagiClaw:
    def __init__(self, id: int = 0, mode: str = "teleoperation") -> None:
        """
        MagiClaw initialization.

        Args:
            id (int): The ID of the claw.
            mode (str): The mode to run MagiClaw in. (default: "teleoperation")
        Raises:
            ValueError: If the claw ID is invalid or not provided.
        """

        # Check if the claw ID is valid
        if id < 0 or id > 1:
            raise ValueError("\033[31mInvalid ID! Must be >= 0.\033[0m")
        if id is None:
            raise ValueError("\033[31mID must be provided!\033[0m")
        # Set the claw ID
        self.id = id

        # Initialize the logger
        log_file_path = f"log/{time.strftime('%Y-%m-%d_%H-%M-%S')}.txt"
        self.logger = init_logger(log_file_path)
        self.logger.info(f"Starting MagiClaw with id={self.id}")

        # Load the claw parameters
        try:
            with open(f"./configs/claw/claw_{self.id}.yaml", "r") as f:
                self.claw_params = yaml.safe_load(f)
        except FileNotFoundError as e:
            self.logger.error(f"Failed to load configuration file: {e}")
            raise e

        # Load the bilateral parameters
        try:
            with open(f"./configs/bilateral.yaml", "r") as f:
                self.bilateral_params = yaml.load(f, Loader=yaml.Loader)
        except FileNotFoundError as e:
            self.logger.error(f"Failed to load configuration file: {e}")
            raise e

        # Store addresses
        self.addrs = self.claw_params["addr"]
        self.bilateral_addr = self.bilateral_params["addr"]

        # Multi-processing
        self.processes = []
        self.mode = mode
        if self.mode not in ["teleoperation", "standalone"]:
            raise ValueError(
                "\033[31mInvalid mode! Must be 'teleoperation' or 'standalone'.\033[0m"
            )
        self.logger.info(f"MagiClaw {self.id} initialized with {self.mode} mode")

    def run(self, loop_rate: int = 30) -> None:
        """
        Run the MagiClaw by starting all processes.
        
        Args:
            loop_rate (int): The loop rate in Hz.
        Raises:
            KeyboardInterrupt: If the user interrupts the process.
            Exception: If any error occurs during the process.
        """

        # Create processes
        if self.mode == "teleoperation":
            self.processes = teleoperation_processes(
                logger=self.logger,
                addrs=self.addrs,
                bilateral_addr=self.bilateral_addr,
                claw_params=self.claw_params,
                loop_rate=loop_rate,
            )
        elif self.mode == "standalone":
            self.processes = standalone_processes(
                logger=self.logger,
                addrs=self.addrs,
                claw_params=self.claw_params,
                loop_rate=loop_rate,
            )

        # Start processes
        try:
            self.logger.info("Starting MagiClaw processes...")
            # Set all processes as daemon except the last one
            for i in range(len(self.processes) - 1):
                self.processes[i].daemon = True
                self.processes[i].start()
                self.logger.info(f"Process {i} started")
                # Small delay to ensure processes start in sequence
                time.sleep(1)

            # Start the publish process (non-daemon)
            self.processes[-1].start()
            self.logger.info("Publish process started")

            # Join the publish process to keep main thread active
            self.processes[-1].join()

        except KeyboardInterrupt:
            self.logger.info("Cleaning up processes due to keyboard interrupt...")
        except Exception as e:
            self.logger.error(f"Error in main process: {str(e)}")
        finally:
            # Clean up all processes
            for i, process in enumerate(self.processes):
                if process.is_alive():
                    self.logger.info(f"Terminating process {i}")
                    process.terminate()
                    process.join(timeout=1.0)
            self.logger.info("MagiClaw terminated.")


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Run MagiClaw")
    parser.add_argument(
        "--id",
        type=int,
        default=0,
        help="The ID of the MagiClaw (default: 0).",
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["teleoperation", "standalone"],
        default="teleoperation",
        help="Mode to run MagiClaw in, either 'teleoperation' or 'standalone'.",
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
