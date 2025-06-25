#!/usr/bin/env python

"""
Utility logging functions for MagiClaw.
"""

import os
import gc
import psutil
import logging
from logging import Logger


def init_logger(log_file_path: str) -> Logger:
    """
    Initialize the logger.

    This function sets up the logging configuration for the MagiClaw.
    It creates a log file with a timestamp and sets the logging level to INFO.
    The log messages will be saved to both the file and the console.

    Args:
        log_file_path (str): The path to the log file.

    Returns:
        logger (logging.Logger): The initialized logger.
    """

    # Create the directory if it does not exist
    if not os.path.exists(os.path.dirname(log_file_path)):
        os.makedirs(os.path.dirname(log_file_path))

    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        handlers=[
            logging.FileHandler(log_file_path),
            logging.StreamHandler(),
        ],
    )

    # Create a logger
    logger = logging.getLogger("MagiClaw")

    # Return the logger
    return logger


def check_system_resources(logger: Logger, fps: str) -> bool:
    """
    Check system resources and return True if they are critical.

    This function checks the CPU temperature, memory usage, and CPU usage.
    If any of these resources are above a certain threshold, it logs a critical message.

    Args:
        logger (logging.Logger): The logger to log messages.
        fps (int): The frames per second to log.

    Returns:
        bool: True if any resource is critical, False otherwise.
    """

    # Check CPU temperature and memory usage
    with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
        cpu_temp = float(f.read()) / 1000.0
    memory_percent = psutil.virtual_memory().percent
    cpu_percent = psutil.cpu_percent(interval=0.1)

    # Log system status every call
    logger.info(
        f"System status: Memory: {memory_percent}%, CPU: {cpu_percent}%, FPS: {fps}"
    )

    gc.collect()  # Run garbage collection to free up memory

    # Check for critical conditions
    if cpu_temp > 80:
        # 80°C is getting dangerous for Pi
        logger.critical(f"CPU temperature critical: {cpu_temp}°C")
        return True
    if memory_percent > 90:
        # Almost out of memory
        logger.critical(f"Memory usage critical: {memory_percent}%")
        return True
    return False
