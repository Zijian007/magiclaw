#!/usr/bin/env python

"""
Run MagiClaw with standalone mode

This script starts the MagiClaw to run in standalone mode.
The MagiClaw consists of a claw, two fingers, and a phone.
Separate processes for each component is going to be created, and they are connected using ZeroMQ.
It pubishes the data to the specified address, including:
    - Claw angle, speed, iq, and temperature
    - Finger image, pose, force, and node
    - Phone color image and depth image

Before running this script, make sure to set up all necessary configurations in `./config/`.

To run the script, use the following command:

```bash
python run_magiclaw.py --claw_id <claw_id>
```

where `<claw_id>` is the ID of the claw to be used, corresponding to the configuration.
"""

import argparse
import multiprocessing
import time
import yaml
from magiclaw.utils.process_utils import (
    standalone_claw_process,
    finger_process,
    publish_process,
)
from magiclaw.utils.logging_utils import init_logger


class MagiClaw:
    def __init__(self, claw_id: int = None) -> None:
        """MagiClaw initialization.

        Args:
            claw_id (int): The ID of the claw.
        """

        # Check if the claw ID is valid
        if claw_id < 0 or claw_id > 1:
            raise ValueError("\033[31mInvalid claw ID! Must be >= 0.\033[0m")
        if claw_id is None:
            raise ValueError("\033[31mClaw ID must be provided!\033[0m")
        # Set the claw ID
        self.claw_id = claw_id

        # Initialize the logger
        log_file_path = f"log/{time.strftime('%Y-%m-%d_%H-%M-%S')}.txt"
        self.logger = init_logger(log_file_path)
        self.logger.info(f"Starting MagiClaw with claw_id={self.claw_id}")

        # Load the claw parameters
        try:
            with open(f"./config/claw/claw_{self.claw_id}.yaml", "r") as f:
                self.claw_params = yaml.load(f, Loader=yaml.Loader)
        except Exception as e:
            self.logger.error(f"Failed to load claw parameters: {str(e)}")
            raise

        # Store addresses
        self.claw_addr = self.claw_params["addr"]["claw"]
        self.finger_0_addr = self.claw_params["addr"]["finger_0"]
        self.finger_1_addr = self.claw_params["addr"]["finger_1"]
        self.publisher_addr = self.claw_params["addr"]["publisher"]

        # Multi-processing
        self.processes = []

    def run(self, loop_rate: int = 60) -> None:
        """Run the MagiClaw by starting all processes."""

        # Create processes
        self.processes.append(
            multiprocessing.Process(
                target=standalone_claw_process,
                args=(self.logger, self.claw_addr, self.claw_params),
            )
        )

        self.processes.append(
            multiprocessing.Process(
                target=finger_process,
                args=(
                    self.logger,
                    0,
                    self.finger_0_addr,
                    self.claw_params["camera_0"]["params_path"],
                    self.claw_params["finger_net_0"],
                    loop_rate,
                ),
            )
        )

        self.processes.append(
            multiprocessing.Process(
                target=finger_process,
                args=(
                    self.logger,
                    1,
                    self.finger_1_addr,
                    self.claw_params["camera_1"]["params_path"],
                    self.claw_params["finger_net_1"],
                    loop_rate,
                ),
            )
        )

        self.processes.append(
            multiprocessing.Process(
                target=publish_process,
                args=(
                    self.logger,
                    self.claw_addr,
                    self.finger_0_addr,
                    self.finger_1_addr,
                    self.publisher_addr,
                    loop_rate,
                ),
            )
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
    # Parse the arguments
    parser = argparse.ArgumentParser(description="Run MagiClaw in standalone mode.")
    parser.add_argument("--claw_id", type=int, default=0, help="The ID of the claw.")
    parser.add_argument(
        "--loop_rate", type=int, default=30, help="The loop rate in Hz."
    )
    args = parser.parse_args()

    # Create the MagiClaw instance
    magiclaw = MagiClaw(args.claw_id)
    # Run the MagiClaw
    magiclaw.run(args.loop_rate)
