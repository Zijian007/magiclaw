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

magiclaw = MagiClaw(id=<id>, mode=<mode>, loop_rate=<loop_rate>, phone_host=<phone_host>, bilateral_host=<bilateral_host>)
magiclaw.run()
```

where `<id> is the MagiClaw's ID (corresponding to the `CAN` interface, default is 0), <mode> is the operation mode (
including `standalone` and `teleop`, default is `standalone`), <loop_rate> is the loop rate in Hz (default is 30),
`<phone_host>` is the host address for the phone (overwrite the default value in the config file, optional),  and
`<bilateral_host>` is the host address for the bilateral communication (only required in `teleop` mode, overwrite the
default value in the config file, optional).
"""

import time
import yaml
import pathlib
from typing import Optional
from .config import ZMQConfig, ClawConfig, CameraConfig, FingerNetConfig, PhoneConfig
from .utils.process_utils import teleop_processes, standalone_processes
from .utils.logging_utils import init_logger


class MagiClaw:
    def __init__(
        self,
        id: int = 0,
        mode: str = "standalone",
        loop_rate: int = 30,
        phone_host: Optional[str] = None,
        bilateral_host: Optional[str] = None,
    ) -> None:
        """
        MagiClaw initialization.

        Args:
            id (int): The ID of the claw.
            mode (str): The mode to run MagiClaw in. (default: "standalone")
            loop_rate (int): The loop rate in Hz. (default: 30)
            phone_host (Optional[str]): The host address for the phone. (default: None)
            bilateral_host (Optional[str]): The host address for the bilateral communication. (default: None)

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

        # Set the mode
        self.mode = mode
        if self.mode not in ["teleop", "standalone"]:
            raise ValueError(
                "\033[31mInvalid mode! Must be 'teleop' or 'standalone'.\033[0m"
            )

        # Set root directory
        self.root_dir = pathlib.Path(__file__).parent.parent

        # Initialize the logger
        log_file_path = self.root_dir.joinpath(
            f"log/{time.strftime('%Y-%m-%d_%H-%M-%S')}.txt"
        )
        self.logger = init_logger(str(log_file_path))
        self.logger.info(f"Starting MagiClaw with id={self.id}")

        # Load the parameters
        try:
            magiclaw_config_path = self.root_dir.joinpath(
                f"configs/magiclaw_{self.id}.yaml"
            )
            with magiclaw_config_path.open("r") as f:
                params = yaml.load(f, Loader=yaml.Loader)

            self.claw_cfg = ClawConfig()
            self.claw_cfg.read_config_file(params["claw"], root_dir=str(self.root_dir))
            self.camera_0_cfg = CameraConfig()
            self.camera_0_cfg.read_config_file(
                params["camera_0"], root_dir=str(self.root_dir)
            )
            self.camera_1_cfg = CameraConfig()
            self.camera_1_cfg.read_config_file(
                params["camera_1"], root_dir=str(self.root_dir)
            )
            self.fingernet_cfg = FingerNetConfig(
                model_path=str(self.root_dir.joinpath(params["fingernet"]))
            )
            self.phone_cfg = PhoneConfig()
            self.phone_cfg.read_config_file(
                params["phone"], root_dir=str(self.root_dir)
            )
            if phone_host is not None:
                self.phone_cfg.set_host(phone_host)
            self.zmq_cfg = ZMQConfig()
        except FileNotFoundError as e:
            self.logger.error(f"Failed to load configuration file: {e}")
            raise e

        # Load the bilateral parameters
        if self.mode == "teleop":
            try:
                bilateral_config_path = self.root_dir.joinpath(
                    f"configs/bilateral.yaml"
                )
                with bilateral_config_path.open("r") as f:
                    bilateral_params = yaml.load(f, Loader=yaml.Loader)
                    if bilateral_host is not None:
                        self.zmq_cfg.set_bilateral_host(bilateral_host)
                    else:
                        self.zmq_cfg.set_bilateral_host(bilateral_params["host"])
            except FileNotFoundError as e:
                self.logger.error(f"Failed to load configuration file: {e}")
                raise e

        # Create processes
        self.processes = []
        if self.mode == "teleop":
            self.processes = teleop_processes(
                logger=self.logger,
                claw_cfg=self.claw_cfg,
                camera_0_cfg=self.camera_0_cfg,
                camera_1_cfg=self.camera_1_cfg,
                fingernet_cfg=self.fingernet_cfg,
                phone_cfg=self.phone_cfg,
                zmq_cfg=self.zmq_cfg,
                mode=bilateral_params["mode"],
                loop_rate=loop_rate,
            )
        else:
            self.processes = standalone_processes(
                logger=self.logger,
                claw_cfg=self.claw_cfg,
                camera_0_cfg=self.camera_0_cfg,
                camera_1_cfg=self.camera_1_cfg,
                fingernet_cfg=self.fingernet_cfg,
                phone_cfg=self.phone_cfg,
                zmq_cfg=self.zmq_cfg,
                loop_rate=loop_rate,
            )

        self.logger.info(f"MagiClaw {self.id} initializing with {self.mode} mode")

    def run(self) -> None:
        """
        Run the MagiClaw by starting all processes.

        Args:
            loop_rate (int): The loop rate in Hz.
        Raises:
            KeyboardInterrupt: If the user interrupts the process.
            Exception: If any error occurs during the process.
        """

        # Start processes
        try:
            self.logger.info("Starting MagiClaw processes...")
            # Set all processes as daemon except the last one
            for i in range(len(self.processes) - 1):
                # self.processes[i].daemon = True
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
