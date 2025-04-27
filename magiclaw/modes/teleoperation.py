#!/usr/bin/env python

"""
Teleoperation
=================

This module generates processes for teleoperation mode.
It creates processes for the claw, two fingers, and a publisher.
- The claw process is to control the claw under the bilateral mode.
- The finger processes are to receive the data from the finger.
- The publisher process is to package the data and publish it to the network.
"""

import multiprocessing
from magiclaw.utils.process_utils import (
    bilateral_claw_process,
    finger_process,
    publish_process,
)


def teleoperation_processes(
    logger,
    addrs: dict,
    bilateral_addr: str,
    claw_params: dict,
    loop_rate: int,
) -> list:
    """
    Generate processes for teleoperation mode.

    Args:
        logger (logging.Logger): The logger object.
        addrs (dict): The addresses for the claw, fingers, and publisher.
        bilateral_addr (str): The address for the bilateral claw.
        claw_params (dict): The parameters for the claw process.
        loop_rate (int): The loop rate in Hz.
    """

    # Create a list to store the processes
    processes = []

    # Create the claw process
    processes.append(
        multiprocessing.Process(
            target=bilateral_claw_process,
            args=(
                logger,
                addrs["claw"],
                bilateral_addr,
                claw_params,
            ),
            name="claw_process",
        )
    )

    # Create the finger 0 process
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
            name="finger_0_process",
        )
    )

    # Create the finger 1 process
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
            name="finger_1_process",
        )
    )

    # Create the publish process
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
            name="publish_process",
        )
    )

    return processes
