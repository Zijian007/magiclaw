#!/usr/bin/env python

"""
Utility functions for process management in MagiClaw.

This module contains functions to manage the claw process, finger process, phone process, and publisher process.
They can be combined to create a complete MagiClaw system in both standalone and teleoperation modes.
"""

import time
import cv2
import numpy as np
from multiprocessing import Process
from logging import Logger
from magiclaw.devices.claw import Claw
from magiclaw.devices.camera import UsbCamera, WebCamera
from magiclaw.models import FingerNet
from magiclaw.modules.zmq import (
    MagiClawPublisher,
    FingerPublisher,
    FingerSubscriber,
    ClawPublisher,
    ClawSubscriber,
    PhoneSubscriber,
)
from magiclaw.config import (
    ZMQConfig,
    ClawConfig,
    CameraConfig,
    FingerNetConfig,
    PhoneConfig,
)
from magiclaw.utils.logging_utils import check_system_resources
from magiclaw.utils.math_utils import convert_pose, r_from_quat, r_to_quat


def standalone_claw_process(
    logger: Logger,
    claw_cfg: ClawConfig,
    zmq_cfg: ZMQConfig,
) -> None:
    """
    Standalone claw process.

    This function initializes the claw and starts the spring damping control loop.
    It reads the motor status and publishes the data to the specified address.

    Args:
        claw_addr: The address of the claw.
        claw_params: The parameters of the claw.
    """

    print("{:=^80}".format(f" MagiClaw {claw_cfg.claw_id} Initialization "))
    # Initialize the claw
    claw = Claw(claw_cfg, mode="standalone")

    # Initialize the publisher with high water mark to limit buffer
    claw_publisher = ClawPublisher(host=zmq_cfg.public_host, port=zmq_cfg.claw_port)

    print(f" MagiClaw {claw_cfg.claw_id} Initialization Done.")
    print("{:=^80}".format(""))

    # Start the loop
    logger.info(f"Claw {claw_cfg.claw_id} process started")

    # Set the loop period
    loop_rate = 200
    loop_start = 0.0
    remaining_time = 0.0

    # Start the control loop
    try:
        while True:
            loop_start = time.time()
            # Control the claw
            claw.spring_damping_control(target_angle=0.0)

            # limit speed, temperature to stop motor
            if abs(claw.motor_speed) > 10000:
                raise ValueError("Motor stopped due to speed limit exceeded.")
            if claw.motor_temperature > 80:
                raise ValueError("Motor stopped due to temperature limit exceeded.")

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
            remaining_time = max(0, (1.0 / loop_rate) - (loop_end - loop_start))
            if remaining_time > 0:
                time.sleep(remaining_time)
            loop_start = time.time()
    except KeyboardInterrupt:
        # Handle keyboard interrupt
        logger.info(f"Claw {claw_cfg.claw_id} process terminated by user")
    except MemoryError:
        # Handle memory error
        logger.error(f"Memory out, cleaning up...")
    except Exception as e:
        # Handle other exceptions
        logger.error(f"Claw process: {str(e)}")
    finally:
        # Release claw resources
        claw.stop()
        # Close ZMQ resources
        claw_publisher.close()
        return


def bilateral_claw_process(
    logger: Logger,
    claw_cfg: ClawConfig,
    zmq_cfg: ZMQConfig,
    mode: str,
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

    print("{:=^80}".format(f" MagiClaw {claw_cfg.claw_id} Initialization "))
    # Initialize the claw
    claw = Claw(
        claw_cfg,
        mode=mode,
    )

    # Initialize the publisher with high water mark to limit buffer
    claw_publisher = ClawPublisher(host=zmq_cfg.public_host, port=zmq_cfg.claw_port)
    if zmq_cfg.bilateral_host is None:
        logger.warning(
            "Bilateral subscriber is not initialized, bilateral data will not be used."
        )
    else:
        bilateral_subscriber = ClawSubscriber(
            host=zmq_cfg.bilateral_host,
            port=zmq_cfg.claw_port,
            timeout=100,
        )

    print(f" MagiClaw {claw_cfg.claw_id} Initialization Done.")
    print("{:=^80}".format(""))

    # Start the loop
    logger.info(f"Claw {claw_cfg.claw_id} process started")

    # Set the loop period
    log_count = 0
    loop_rate = 200
    loop_start = 0.0
    remaining_time = 0.0

    # Start the control loop
    try:
        while True:
            loop_start = time.time()

            # Get bilateral motor status
            try:
                _, _, bilateral_motor_angle_percent, bilateral_motor_speed, _, _ = (
                    bilateral_subscriber.subscribeMessage()
                )
            except Exception as e:
                # Handle exception and log error
                log_count += 1
                if log_count == loop_rate:
                    logger.warning(f"Bilateral subscriber: {str(e)}")
                    log_count = 0
                # If no message is received, set default values
                bilateral_motor_angle_percent = None
                bilateral_motor_speed = None

            # Control the claw
            claw.bilateral_spring_damping_control(
                bilateral_motor_angle_percent=bilateral_motor_angle_percent,
                bilateral_motor_speed=bilateral_motor_speed,
            )

            # limit speed, temperature to stop motor
            if abs(claw.motor_speed) > 10000:
                raise ValueError("Motor stopped due to speed limit exceeded.")
            if claw.motor_temperature > 80:
                raise ValueError("Motor stopped due to temperature limit exceeded.")

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
            remaining_time = max(0, (1.0 / loop_rate) - (loop_end - loop_start))
            if remaining_time > 0:
                time.sleep(remaining_time)
    except KeyboardInterrupt:
        # Handle keyboard interrupt
        logger.info(f"Claw {claw_cfg.claw_id} process terminated by user")
    except MemoryError:
        # Handle memory error
        logger.error(f"Memory out, cleaning up...")
    except Exception as e:
        # Handle other exceptions
        logger.error(f"Claw process: {str(e)}")
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
    camera_cfg: CameraConfig,
    fingernet_cfg: FingerNetConfig,
    zmq_cfg: ZMQConfig,
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

    # Initialize camera based on the mode
    if camera_cfg.mode == "usb":
        camera = UsbCamera(
            name=f"camera_{finger_index}",
            camera_cfg=camera_cfg,
        )
    elif camera_cfg.mode == "web":
        camera = WebCamera(
            name=f"camera_{finger_index}",
            camera_cfg=camera_cfg,
        )
    else:
        raise ValueError("\033[31mUnsupported camera mode!\033[0m")

    # Initialize finger net
    fingernet = FingerNet(fingernet_cfg)

    # Initialize publisher with high water mark
    finger_publisher = FingerPublisher(
        host=zmq_cfg.public_host,
        port=getattr(zmq_cfg, f"finger_{finger_index}_port"),
    )

    print(f" Finger {finger_index} Initialization Done.")
    print("{:=^80}".format(""))

    # JPEG compression - higher value = lower quality = less memory
    jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), 50]

    # Set the loop period
    loop_start = 0.0
    remaining_time = 0.0

    # Start the loop
    logger.info(f"Finger {finger_index} process started")
    try:
        while True:
            loop_start = time.time()

            # Get the image and pose
            pose, img = camera.readImageAndPose()
            if pose[0, 0] > 999:
                pose = np.zeros_like(pose)
            # Convert the pose to the reference pose
            pose_ref = camera.poseToReferece(pose)
            # Convert the pose from the marker frame to the camera frame
            pose_global = camera.poseAxisTransfer(pose_ref)
            # Convert the pose to the euler angles
            pose_euler = camera.poseVectorToEuler(pose_global)

            # Predict the force and node
            force, node = fingernet.infer(pose_euler)

            # Publish the data
            finger_publisher.publishMessage(
                img_bytes=cv2.imencode(".jpg", img, jpeg_params)[1].tobytes(),
                pose=pose_euler.flatten().tolist(),
                force=force.flatten().tolist(),
                node=node.flatten().tolist(),
            )

            # Fix the loop time
            loop_end = time.time()
            remaining_time = max(0, (1.0 / loop_rate) - (loop_end - loop_start))
            if remaining_time > 0:
                time.sleep(remaining_time)
    except KeyboardInterrupt:
        logger.info(f"Finger {finger_index} process terminated by user")
    except MemoryError:
        logger.error(f"Memory out, cleaning up...")
    except Exception as e:
        logger.error(f"Finger {finger_index} process: {str(e)}")
    finally:
        # Release camera resources
        camera.release()
        # Close ZMQ resources
        finger_publisher.close()
        return


def publish_process(
    logger: Logger,
    phone_cfg: PhoneConfig,
    zmq_cfg: ZMQConfig,
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
    claw_subscriber = ClawSubscriber(
        host=zmq_cfg.public_host,
        port=zmq_cfg.claw_port,
        timeout=int(1000 / loop_rate),
    )
    finger_0_subscriber = FingerSubscriber(
        host=zmq_cfg.public_host,
        port=zmq_cfg.finger_0_port,
        timeout=int(1000 / loop_rate),
    )
    finger_1_subscriber = FingerSubscriber(
        host=zmq_cfg.public_host,
        port=zmq_cfg.finger_1_port,
        timeout=int(1000 / loop_rate),
    )
    if phone_cfg.host is None:
        logger.warning(
            "Phone subscriber is not initialized, phone data will be default."
        )
    else:
        phone_subscriber = PhoneSubscriber(
            host=phone_cfg.host,
            port=phone_cfg.port,
            timeout=int(1000 / loop_rate),
        )
    magiclaw_publisher = MagiClawPublisher(
        host=zmq_cfg.public_host,
        port=zmq_cfg.publish_port,
    )

    claw_angle = 0.0
    motor_angle = 0.0
    motor_angle_percent = 0.0
    motor_speed = 0.0
    motor_iq = 0.0
    motor_temperature = 0

    finger_0_pose = np.zeros(6, dtype=np.float32).tolist()
    finger_0_force = np.zeros(6, dtype=np.float32).tolist()
    finger_0_node = np.zeros(6, dtype=np.float32).tolist()
    finger_0_img_bytes = cv2.imencode(
        ".jpg",
        np.zeros((240, 320, 3), dtype=np.uint8),
        [int(cv2.IMWRITE_JPEG_QUALITY), 50],
    )[1].tobytes()
    finger_1_pose = np.zeros(6, dtype=np.float32).tolist()
    finger_1_force = np.zeros(6, dtype=np.float32).tolist()
    finger_1_node = np.zeros(6, dtype=np.float32).tolist()
    finger_1_img_bytes = cv2.imencode(
        ".jpg",
        np.zeros((240, 320, 3), dtype=np.uint8),
        [int(cv2.IMWRITE_JPEG_QUALITY), 50],
    )[1].tobytes()
    phone_color_img_bytes = cv2.imencode(
        ".jpg",
        np.zeros((480, 640, 3), dtype=np.uint8),
        [int(cv2.IMWRITE_JPEG_QUALITY), 50],
    )[1].tobytes()
    phone_depth_img_bytes = np.ones((192, 256), dtype=np.uint16).tobytes()
    phone_depth_width = 256
    phone_depth_height = 192
    phone_local_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    phone_global_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    magiclaw_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

    print(" MagiClaw Publisher Initialization Done.")
    print("{:=^80}".format(""))

    # Set the initial time and message count
    start_time = 0
    msg_count = 0

    # y-up, x-right, z-back to z-up, x-back, y-right
    phone_pose_conv_mat = np.array(
        [
            [1, 0, 0, 0],
            [0, 0, -1, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1],
        ],
        dtype=np.float32,
    )

    phone_init_r = r_from_quat(np.array(phone_cfg.init_pose[3:], dtype=np.float32))

    # Start the loop
    try:
        while True:
            # Get claw data
            try:
                (
                    claw_angle,
                    motor_angle,
                    motor_angle_percent,
                    motor_speed,
                    motor_iq,
                    motor_temperature,
                ) = claw_subscriber.subscribeMessage()
            except Exception as e:
                if msg_count % (loop_rate * 2) == 0:
                    logger.warning(f"Claw subscriber: {str(e)}")

            # Get finger 0 data
            try:
                finger_0_img_bytes, finger_0_pose, finger_0_force, finger_0_node = (
                    finger_0_subscriber.subscribeMessage()
                )
            except Exception as e:
                if msg_count % (loop_rate * 2) == 0:
                    logger.warning(f"Finger 0 subscriber: {str(e)}")

            # Get finger 1 data
            try:
                (
                    finger_1_img_bytes,
                    finger_1_pose[:],
                    finger_1_force[:],
                    finger_1_node[:],
                ) = finger_1_subscriber.subscribeMessage()
            except Exception as e:
                if msg_count % (loop_rate * 2) == 0:
                    logger.warning(f"Finger 1 subscriber: {str(e)}")

            # Get phone data
            try:
                (
                    phone_color_img_bytes,
                    phone_depth_img_bytes,
                    phone_depth_width,
                    phone_depth_height,
                    phone_local_pose[:],
                    phone_global_pose[:],
                ) = phone_subscriber.subscribeMessage()

                # Change phone pose from y-up to z-up
                phone_local_pose[:] = list(
                    convert_pose(
                        np.array(phone_local_pose, dtype=np.float32),
                        phone_pose_conv_mat,
                    )
                )
                phone_global_pose[:] = list(
                    convert_pose(
                        np.array(phone_global_pose, dtype=np.float32),
                        phone_pose_conv_mat,
                    )
                )

                phone_curr_r = r_from_quat(
                    np.array(phone_global_pose[3:], dtype=np.float32)
                )
                phone_delta_r = phone_curr_r * phone_init_r.inv()
                magiclaw_quat = r_to_quat(phone_delta_r)
                magiclaw_pose[:] = [
                    phone_global_pose[0],
                    phone_global_pose[1],
                    phone_global_pose[2],
                    magiclaw_quat[0],
                    magiclaw_quat[1],
                    magiclaw_quat[2],
                    magiclaw_quat[3],
                ]
            except Exception as e:
                if msg_count % (loop_rate * 2) == 0:
                    logger.warning(f"Phone subscriber: {str(e)}")

            # Publish the data
            magiclaw_publisher.publishMessage(
                claw_angle=claw_angle,
                motor_angle=motor_angle,
                motor_speed=motor_speed,
                motor_iq=motor_iq,
                motor_temperature=motor_temperature,
                finger_0_img_bytes=finger_0_img_bytes,
                finger_0_pose=finger_0_pose,
                finger_0_force=finger_0_force,
                finger_0_node=finger_0_node,
                finger_1_img_bytes=finger_1_img_bytes,
                finger_1_pose=finger_1_pose,
                finger_1_force=finger_1_force,
                finger_1_node=finger_1_node,
                phone_color_img_bytes=phone_color_img_bytes,
                phone_depth_img_bytes=phone_depth_img_bytes,
                phone_depth_width=phone_depth_width,
                phone_depth_height=phone_depth_height,
                phone_local_pose=phone_local_pose,
                phone_global_pose=phone_global_pose,
                magiclaw_pose=magiclaw_pose,
            )

            # Check resources every loop_rate * 2 messages
            if msg_count % (loop_rate * 2) == 0:
                if check_system_resources(
                    logger=logger,
                    fps=f"{(loop_rate * 2 / (time.time() - start_time)): .2f}",
                ):
                    logger.warning(
                        "System resources critical in publish process, stopping"
                    )
                    return
                start_time = time.time()
                msg_count = 0

            # Increment the message count
            msg_count += 1
    except KeyboardInterrupt:
        logger.info("Publish process terminated by user")
    except MemoryError:
        logger.error("Memory out, cleaning up...")
    except Exception as e:
        logger.error(f"Publish process: {str(e)}")
    finally:
        # Close all ZMQ resources
        claw_subscriber.close()
        finger_0_subscriber.close()
        finger_1_subscriber.close()
        phone_subscriber.close() if phone_subscriber else None
        magiclaw_publisher.close()
        return


def standalone_processes(
    logger,
    claw_cfg: ClawConfig,
    camera_0_cfg: CameraConfig,
    camera_1_cfg: CameraConfig,
    fingernet_cfg: FingerNetConfig,
    phone_cfg: PhoneConfig,
    zmq_cfg: ZMQConfig,
    loop_rate: int,
) -> list:
    """
    Generate processes for standalone mode.

    Args:
        logger (logging.Logger): The logger object.
        zmq_cfg (ZMQConfig): The ZMQ configuration object.
        claw_cfg (ClawConfig): The claw configuration object.
        camera_cfg (CameraConfig): The camera configuration object.
        fingernet_cfg (FingerNetConfig): The finger net configuration object.
        loop_rate (int): The loop rate in Hz.
    """

    # Create a list to store the processes
    processes = []

    # Create processes
    processes.append(
        Process(
            target=standalone_claw_process,
            args=(logger, claw_cfg, zmq_cfg),
        )
    )

    # Create the finger 0 process
    processes.append(
        Process(
            target=finger_process,
            args=(logger, 0, camera_0_cfg, fingernet_cfg, zmq_cfg, loop_rate),
            name="finger_0_process",
        )
    )

    # Create the finger 1 process
    processes.append(
        Process(
            target=finger_process,
            args=(logger, 1, camera_1_cfg, fingernet_cfg, zmq_cfg, loop_rate),
            name="finger_1_process",
        )
    )

    # Create the publish process
    processes.append(
        Process(
            target=publish_process,
            args=(logger, phone_cfg, zmq_cfg, loop_rate),
            name="publish_process",
        )
    )

    return processes


def teleop_processes(
    logger,
    claw_cfg: ClawConfig,
    camera_0_cfg: CameraConfig,
    camera_1_cfg: CameraConfig,
    fingernet_cfg: FingerNetConfig,
    phone_cfg: PhoneConfig,
    zmq_cfg: ZMQConfig,
    mode: str,
    loop_rate: int,
) -> list:
    """
    Generate processes for teleoperation mode.

    Args:
        logger (logging.Logger): The logger object.
        zmq_cfg (ZMQConfig): The ZMQ configuration object.
        claw_cfg (ClawConfig): The claw configuration object.
        camera_cfg (CameraConfig): The camera configuration object.
        fingernet_cfg (FingerNetConfig): The finger net configuration object.
        mode (str): The mode to run the claw in, "leader" or "follower".
        loop_rate (int): The loop rate in Hz.
    """

    # Create a list to store the processes
    processes = []

    # Create the claw process
    processes.append(
        Process(
            target=bilateral_claw_process,
            args=(logger, claw_cfg, zmq_cfg, mode),
            name="claw_process",
        )
    )

    # Create the finger 0 process
    processes.append(
        Process(
            target=finger_process,
            args=(logger, 0, camera_0_cfg, fingernet_cfg, zmq_cfg, loop_rate),
            name="finger_0_process",
        )
    )

    # Create the finger 1 process
    processes.append(
        Process(
            target=finger_process,
            args=(logger, 1, camera_1_cfg, fingernet_cfg, zmq_cfg, loop_rate),
            name="finger_1_process",
        )
    )

    # Create the publish process
    processes.append(
        Process(
            target=publish_process,
            args=(logger, phone_cfg, zmq_cfg, loop_rate),
            name="publish_process",
        )
    )

    return processes
