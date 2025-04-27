#!/usr/bin/env python

"""
Utility functions for process management.

This module contains functions to manage the claw process, finger process, phone process, and publisher process.
They can be combined to create a complete MagiClaw system in both standalone and teleoperation modes.
"""

import time
import yaml
import cv2
import numpy as np
from logging import Logger
from magiclaw.devices.claw import Claw
from magiclaw.devices.camera import UsbCamera, WebCamera
from magiclaw.models.fingernet import FingerNet
from magiclaw.modules.zmq import (
    MagiClawPublisher,
    FingerPublisher,
    FingerSubscriber,
    ClawPublisher,
    ClawSubscriber,
)
from magiclaw.utils.logging_utils import check_system_resources


def standalone_claw_process(
    logger: Logger, claw_addr: dict, claw_params: dict, loop_rate: int = 200
) -> None:
    """
    Standalone claw process.

    This function initializes the claw and starts the spring damping control loop.
    It reads the motor status and publishes the data to the specified address.

    Args:
        claw_addr: The address of the claw.
        claw_params: The parameters of the claw.
    """

    print("{:=^80}".format(f" MagiClaw {claw_params['id']} Initialization "))
    # Initialize the claw
    claw = Claw(
        claw_id=claw_params["id"],
        lead=claw_params["lead"],
        gear_radius=claw_params["gear_radius"],
        **claw_params["motor"],
    )

    # Initialize the publisher with high water mark to limit buffer
    claw_publisher = ClawPublisher(ip=claw_addr["ip"], port=claw_addr["port"])

    print(f" MagiClaw {claw_params['id']} Initialization Done.")
    print("{:=^80}".format(""))

    # Start the loop
    logger.info(f"Claw {claw_params['id']} process started")

    # Set the loop period
    loop_start = time.time()
    elapsed_time = 0.0
    remaining_time = 0.0

    # Start the control loop
    try:
        while True:
            loop_start = time.time()
            # Control the claw
            claw.spring_damping_control(target_angle=0.0)

            # limit speed, temperature to stop motor
            if abs(claw.motor_speed) > 7200 or claw.motor_temperature > 80:
                print("Motor stopped due to limit exceeded.")
                raise ValueError("Motor stopped due to limit exceeded.")

            # Publish the data
            claw_publisher.publishMessage(
                claw_angle=claw.claw_angle,
                motor_angle=claw.motor_angle,
                motor_angle_percent=claw.motor_angle_percent,
                motor_speed=claw.motor_speed,
                motor_iq=claw.motor_iq,
                motor_temperature=claw.motor_temperature,
            )

            # Fix the loop time
            loop_end = time.time()
            elapsed_time = loop_end - loop_start
            remaining_time = max(0, (1.0 / loop_rate) - elapsed_time)
            if remaining_time > 0:
                time.sleep(remaining_time)
            loop_start = time.time()
    except KeyboardInterrupt:
        # Handle keyboard interrupt
        logger.info(f"Claw {claw_params['id']} process terminated by user")
    except MemoryError:
        # Handle memory error
        logger.error(f"Memory error in claw process, cleaning up...")
    except Exception as e:
        # Handle other exceptions
        logger.error(f"Error in claw process: {str(e)}")
    finally:
        # Release claw resources
        claw.stop()
        # Close ZMQ resources
        claw_publisher.close()
        return


def bilateral_claw_process(
    logger: Logger,
    claw_addr: dict,
    bilateral_addr: dict,
    claw_params: dict,
    loop_rate: int = 200,
) -> None:
    """
    Bilateral claw process.

    This function initializes the claw and starts the bilateral spring damping control loop.
    It reads the motor status, subscribes to the bilateral motor status, and publishes the
    data to the specified address.

    Args:
        claw_addr: The address of the claw.
        bilateral_addr: The address of the bilateral claw.
        claw_params: The parameters of the claw.
    """

    print("{:=^80}".format(f" MagiClaw {claw_params['id']} Initialization "))
    # Initialize the claw
    claw = Claw(
        claw_id=claw_params["id"],
        lead=claw_params["lead"],
        gear_radius=claw_params["gear_radius"],
        **claw_params["motor"],
    )

    # Initialize the publisher with high water mark to limit buffer
    claw_publisher = ClawPublisher(ip=claw_addr["ip"], port=claw_addr["port"])
    bilateral_subscriber = ClawSubscriber(
        ip=bilateral_addr["ip"], port=claw_addr["port"]
    )

    print(f" MagiClaw {claw_params['id']} Initialization Done.")
    print("{:=^80}".format(""))

    # Start the loop
    logger.info(f"Claw {claw_params['id']} process started")

    # Set the loop period
    loop_start = time.time()
    elapsed_time = 0.0
    remaining_time = 0.0
    log_count = 0

    # Start the control loop
    try:
        while True:
            loop_start = time.time()

            # Get bilateral motor status
            try:
                _, _, bilateral_motor_angle_percent, bilateral_motor_speed, _, _ = (
                    bilateral_subscriber.subscribeMessage(timeout=5)
                )
            except Exception as e:
                # Handle exception and log error
                log_count += 1
                if log_count == loop_rate:
                    logger.error(f"Error in bilateral subscriber: {str(e)}")
                    log_count = 0
                # If no message is received, set default values
                bilateral_motor_angle_percent = None
                bilateral_motor_speed = None

            # Control the claw
            claw.bilateral_spring_damping_control(
                bilateral_motor_angle_percent=bilateral_motor_angle_percent,
                bilateral_motor_speed=bilateral_motor_speed,
                target_angle=0.0,
            )

            # limit speed, temperature to stop motor
            if abs(claw.motor_speed) > 7200 or claw.motor_temperature > 80:
                print("Motor stopped due to limit exceeded.")
                raise ValueError("Motor stopped due to limit exceeded.")

            # Publish the data
            claw_publisher.publishMessage(
                claw_angle=claw.claw_angle,
                motor_angle=claw.motor_angle,
                motor_angle_percent=claw.motor_angle_percent,
                motor_speed=claw.motor_speed,
                motor_iq=claw.motor_iq,
                motor_temperature=claw.motor_temperature,
            )

            # Fix the loop time
            loop_end = time.time()
            elapsed_time = loop_end - loop_start
            remaining_time = max(0, (1.0 / loop_rate) - elapsed_time)
            if remaining_time > 0:
                time.sleep(remaining_time)
    except KeyboardInterrupt:
        # Handle keyboard interrupt
        logger.info(f"Claw {claw_params['id']} process terminated by user")
    except MemoryError:
        # Handle memory error
        logger.error(f"Memory error in claw process, cleaning up...")
    except Exception as e:
        # Handle other exceptions
        logger.error(f"Error in claw process: {str(e)}")
    finally:
        # Release claw resources
        claw.stop()
        # Close ZMQ resources
        claw_publisher.close()
        bilateral_subscriber.close()
        return


def finger_process(
    logger: Logger,
    finger_index: int,
    finger_addr: dict,
    camera_params_path: str,
    finger_net_params: dict,
    loop_rate: int = 60,
) -> None:
    """
    Finger process.

    This function initializes the camera and finger net, and starts the inference loop.
    It reads the image and aruco pose from the camera, infer force and node.
    It publishes the data to the specified address.

    Args:
        finger_index: The index of the finger.
        finger_addr: The address of the finger.
        camera_params_path: The path to the camera parameters file.
        finger_net_params: The parameters of the finger net.
    """

    print("{:=^80}".format(f" Finger {finger_index} Initialization "))

    # Initialize camera
    with open(camera_params_path, "r") as f:
        camera_params = yaml.load(f.read(), Loader=yaml.Loader)

    # Load detector parameters
    with open(f"./configs/detector.yaml", "r") as f:
        detector_params = yaml.load(f, Loader=yaml.Loader)

    # Initialize camera based on the mode
    if camera_params["mode"] == "usb":
        camera = UsbCamera(
            name=f"camera_{finger_index}",
            camera_params=camera_params,
            detector_params=detector_params,
        )
    elif camera_params["mode"] == "web":
        camera = WebCamera(
            name=f"camera_{finger_index}",
            camera_params=camera_params,
            detector_params=detector_params,
        )
    else:
        raise ValueError("\033[31mUnsupported camera mode!\033[0m")

    # Initialize finger net
    finger_net = FingerNet(
        name=finger_net_params["name"], model_path=finger_net_params["model_path"]
    )

    # Initialize publisher with high water mark
    finger_publisher = FingerPublisher(ip=finger_addr["ip"], port=finger_addr["port"])

    print(f" Finger {finger_index} Initialization Done.")
    print("{:=^80}".format(""))

    # JPEG compression - higher value = lower quality = less memory
    jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), 50]

    # Set the loop period
    loop_start = time.time()
    elapsed_time = 0.0
    remaining_time = 0.0

    # Start the loop
    logger.info(f"Finger {finger_index} process started")
    try:
        while True:
            loop_start = time.time()

            # Get the image and pose
            pose, img = camera.readImageAndPose()
            # Convert the pose to the reference pose
            pose_ref = camera.poseToReferece(pose)
            # Convert the pose from the marker frame to the camera frame
            pose_global = camera.poseAxisTransfer(pose_ref)
            # Convert the pose to the euler angles
            pose_euler = camera.poseVectorToEuler(pose_global)

            # Predict the force and node
            force, node = finger_net.infer(pose_euler)

            # Publish the data
            finger_publisher.publishMessage(
                img_bytes=cv2.imencode(".jpg", img, jpeg_params)[1].tobytes(),
                pose=pose_euler,
                force=force,
                node=node,
            )

            # Fix the loop time
            loop_end = time.time()
            elapsed_time = loop_end - loop_start
            remaining_time = max(0, (1.0 / loop_rate) - elapsed_time)
            if remaining_time > 0:
                time.sleep(remaining_time)
    except KeyboardInterrupt:
        logger.info(f"Finger {finger_index} process terminated by user")
    except MemoryError:
        logger.error(f"Memory error in finger {finger_index} process, cleaning up...")
    except Exception as e:
        logger.error(f"Error in finger {finger_index} process: {str(e)}")
    finally:
        # Release camera resources
        camera.release()
        # Close ZMQ resources
        finger_publisher.close()
        return


def publish_process(
    logger: Logger,
    claw_addr: dict,
    finger_0_addr: dict,
    finger_1_addr: dict,
    publisher_addr: dict,
    loop_rate: int = 60,
) -> None:
    """
    Publish process.

    This function initializes the subscribers and publisher, and starts the publish loop.
    It subscribes to the claw and finger data, and publishes the data to the specified address.

    Args:
        claw_addr: The address of the claw.
        finger_0_addr: The address of the first finger.
        finger_1_addr: The address of the second finger.
        publisher_addr: The address of the publisher.
    """

    print("{:=^80}".format(" MagiClaw Publisher Initialization "))

    # Initialize subscribers with settings to only get latest message
    claw_subscriber = ClawSubscriber(ip=claw_addr["ip"], port=claw_addr["port"])
    finger_0_subscriber = FingerSubscriber(
        ip=finger_0_addr["ip"], port=finger_0_addr["port"]
    )
    finger_1_subscriber = FingerSubscriber(
        ip=finger_1_addr["ip"], port=finger_1_addr["port"]
    )
    magiclaw_publisher = MagiClawPublisher(
        ip=publisher_addr["ip"], port=publisher_addr["port"]
    )

    print(" MagiClaw Publisher Initialization Done.")
    print("{:=^80}".format(""))

    # Set the initial time and message count
    start_time = time.time()
    msg_count = 0

    # Start the loop
    try:
        while True:
            # Get claw data
            try:
                claw_angle, motor_angle, motor_angle_percent, motor_speed, motor_iq, motor_temperature = (
                    claw_subscriber.subscribeMessage()
                )
            except Exception as e:
                logger.error(f"Error in claw subscriber: {str(e)}")
                claw_angle = 0.0
                motor_angle = 0.0
                motor_angle_percent = 0.0
                motor_speed = 0.0
                motor_iq = 0.0
                motor_temperature = 0.0

            # Get finger 0 data
            try:
                img_0_bytes, pose_0, force_0, node_0 = (
                    finger_0_subscriber.subscribeMessage()
                )
            except Exception as e:
                logger.error(f"Error in finger 0 subscriber: {str(e)}")
                img_0_bytes = b""
                pose_0 = np.zeros(6, dtype=np.float32)
                force_0 = np.zeros(6, dtype=np.float32)
                node_0 = np.zeros(6, dtype=np.float32)

            # Get finger 1 data
            try:
                img_1_bytes, pose_1, force_1, node_1 = (
                    finger_1_subscriber.subscribeMessage()
                )
            except Exception as e:
                logger.error(f"Error in finger 1 subscriber: {str(e)}")
                img_1_bytes = b""
                pose_1 = np.zeros(6, dtype=np.float32)
                force_1 = np.zeros(6, dtype=np.float32)
                node_1 = np.zeros(6, dtype=np.float32)

            # Publish the data
            magiclaw_publisher.publishMessage(
                claw_angle=claw_angle,
                motor_angle=motor_angle,
                motor_speed=motor_speed,
                motor_iq=motor_iq,
                motor_temperature=motor_temperature,
                img_0_bytes=img_0_bytes,
                pose_0=pose_0,
                force_0=force_0,
                node_0=node_0,
                img_1_bytes=img_1_bytes,
                pose_1=pose_1,
                force_1=force_1,
                node_1=node_1,
            )

            # Increment the message count
            msg_count += 1

            # Check resources every loop_rate * 2 messages
            if msg_count >= loop_rate * 2:
                if check_system_resources(
                    logger=logger,
                    fps="%.2f" % (loop_rate * 2 / (time.time() - start_time)),
                ):
                    logger.warning(
                        "System resources critical in publish process, stopping"
                    )
                    # Explicitly clean up
                    claw_subscriber.close()
                    finger_0_subscriber.close()
                    finger_1_subscriber.close()
                    magiclaw_publisher.close()
                    return
                start_time = time.time()
                msg_count = 0

            # Control loop rate
            time.sleep(max(0, 1.0 / loop_rate - 0.01))  # Small margin for processing

    except KeyboardInterrupt:
        logger.info("Publish process terminated by user")
    except MemoryError:
        logger.error("Memory error in publish process, cleaning up...")
    except Exception as e:
        logger.error(f"Error in publish process: {str(e)}")
    finally:
        # Close all ZMQ resources
        claw_subscriber.close()
        finger_0_subscriber.close()
        finger_1_subscriber.close()
        magiclaw_publisher.close()
        return
