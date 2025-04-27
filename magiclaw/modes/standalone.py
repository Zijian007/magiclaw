#!/usr/bin/env python

"""
Standalone
=================

This module generates processes for standalone mode.
It creates processes for the claw, two fingers, and a publisher.
- The claw process is to control the claw under the standalone mode.
- The finger processes are to receive the data from the finger.
- The publisher process is to package the data and publish it to the network.
"""

import multiprocessing
from magiclaw.utils.process_utils import (
    standalone_claw_process,
    finger_process,
    publish_process,
)


def standalone_processes(
    logger,
    addrs: dict,
    claw_params: dict,
    loop_rate: int) -> list:
    """
    Generate processes for standalone mode.
    
    Args:
        logger (logging.Logger): The logger object.
        addrs (dict): The addresses for the claw, fingers, and publisher.
        claw_params (dict): The parameters for the claw process.
        loop_rate (int): The loop rate in Hz.
    """
    
    # Create a list to store the processes
    processes = []

    # Create processes
    processes.append(
        multiprocessing.Process(
            target=standalone_claw_process,
            args=(logger, addrs["claw"], claw_params),
        )
    )

    processes.append(
        multiprocessing.Process(
            target=finger_process,
            args=(
                logger,
                0,
                addrs["finger_0"],
                claw_params["camera_0"]["params_path"],
                claw_params["finger_net_0"],
                loop_rate,
            ),
        )
    )

    processes.append(
        multiprocessing.Process(
            target=finger_process,
            args=(
                logger,
                1,
                addrs["finger_1"],
                claw_params["camera_1"]["params_path"],
                claw_params["finger_net_1"],
                loop_rate,
            ),
        )
    )

    processes.append(
        multiprocessing.Process(
            target=publish_process,
            args=(
                logger,
                addrs["claw"],
                addrs["finger_0"],
                addrs["finger_1"],
                addrs["publisher"],
                loop_rate,
            ),
        )
    )
