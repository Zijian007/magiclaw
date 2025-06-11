#!/usr/bin/env python

import zmq
import numpy as np
from typing import Tuple
from datetime import datetime
from magiclaw.modules.protobuf import magiclaw_msg_pb2


class MagiClawPublisher:
    def __init__(
        self, host: str, port: int, hwm: int = 1, conflate: bool = True
    ) -> None:
        """Publisher initialization.

        Args:
            host (str): The host address of the publisher.
            port (int): The port number of the publisher.
            hwm (int): High water mark for the publisher. Default is 1.
            conflate (bool): Whether to conflate messages. Default is True.
        """

        print("{:-^80}".format(" Claw Publisher Initialization "))
        print(f"Address: tcp://{host}:{port}")

        # Create a ZMQ context
        self.context = zmq.Context()
        # Create a ZMQ publisher
        self.publisher = self.context.socket(zmq.PUB)
        # Set high water mark
        self.publisher.set_hwm(hwm)
        # Set conflate
        self.publisher.setsockopt(zmq.CONFLATE, conflate)
        # Bind the address
        self.publisher.bind(f"tcp://{host}:{port}")

        # Init the message
        self.magiclaw = magiclaw_msg_pb2.MagiClaw()

        print("Package Claw")
        print("Message Finger")
        print(
            "{\n\tbytes img = 1;\n\trepeated float pose = 2;\n\trepeated float force = 3;\n\trepeated float node = 4;\n}"
        )
        print("Message Phone")
        print("{\n\tbytes color_img = 1;\n\trepeated int32 depth_img = 2\n}")
        print("Message Claw")
        print(
            "{\n\tdouble timestamp = 1;\n\tfloat angle = 2;\n\tfloat speed = 3;\n\tfloat iq = 4;\n\tFinger finger_0 = 5;\n\tFinger finger_1 = 6;\n}"
        )

        print("Claw Publisher Initialization Done.")
        print("{:-^80}".format(""))

    def publishMessage(
        self,
        claw_angle: float = 0.0,
        motor_angle: float = 0.0,
        motor_speed: float = 0.0,
        motor_iq: float = 0.0,
        motor_temperature: int = 0,
        finger_0_img_bytes: bytes = b"",
        finger_0_pose: list = np.zeros(6, dtype=np.float32).tolist(),
        finger_0_force: list = np.zeros(6, dtype=np.float32).tolist(),
        finger_0_node: list = np.zeros(6, dtype=np.float32).tolist(),
        finger_1_img_bytes: bytes = b"",
        finger_1_pose: list = np.zeros(6, dtype=np.float32).tolist(),
        finger_1_force: list = np.zeros(6, dtype=np.float32).tolist(),
        finger_1_node: list = np.zeros(6, dtype=np.float32).tolist(),
        phone_color_img_bytes: bytes = b"",
        phone_depth_img_bytes: bytes = b"",
        phone_depth_width: int = 0,
        phone_depth_height: int = 0,
        phone_local_pose: list = np.zeros(6, dtype=np.float32).tolist(),
        phone_global_pose: list = np.zeros(6, dtype=np.float32).tolist(),
    ) -> None:
        """Publish the message.

        Args:
            img: The image captured by the camera.
            pose: The pose of the marker.
            force: The force on the bottom surface of the finger.
            node: The node displacement of the finger.
        """

        # Set the message
        self.magiclaw.timestamp = datetime.now().timestamp()
        self.magiclaw.claw.angle = claw_angle
        self.magiclaw.claw.motor.angle = motor_angle
        self.magiclaw.claw.motor.speed = motor_speed
        self.magiclaw.claw.motor.iq = motor_iq
        self.magiclaw.claw.motor.temperature = motor_temperature
        self.magiclaw.finger_0.img = finger_0_img_bytes
        self.magiclaw.finger_0.pose[:] = finger_0_pose
        self.magiclaw.finger_0.force[:] = finger_0_force
        self.magiclaw.finger_0.node[:] = finger_0_node
        self.magiclaw.finger_1.img = finger_1_img_bytes
        self.magiclaw.finger_1.pose[:] = finger_1_pose
        self.magiclaw.finger_1.force[:] = finger_1_force
        self.magiclaw.finger_1.node[:] = finger_1_node
        self.magiclaw.phone.color_img = phone_color_img_bytes
        self.magiclaw.phone.depth_img = phone_depth_img_bytes
        self.magiclaw.phone.depth_width = phone_depth_width
        self.magiclaw.phone.depth_height = phone_depth_height
        self.magiclaw.phone.local_pose[:] = phone_local_pose
        self.magiclaw.phone.global_pose[:] = phone_global_pose

        # Publish the message
        self.publisher.send(self.magiclaw.SerializeToString())

    def close(self):
        """Close ZMQ socket and context to prevent memory leaks."""
        if hasattr(self, "publisher") and self.publisher:
            self.publisher.close()
        if hasattr(self, "context") and self.context:
            self.context.term()


class MagiClawSubscriber:
    def __init__(
        self, host: str, port: int, hwm: int = 1, conflate: bool = True
    ) -> None:
        """Subscriber initialization.

        Args:
            host (str): The host address of the subscriber.
            port (int): The port number of the subscriber.
            hwm (int): High water mark for the subscriber. Default is 1.
            conflate (bool): Whether to conflate messages. Default is True.
        """

        print("{:-^80}".format(" Claw Subscriber Initialization "))
        print(f"Address: tcp://{host}:{port}")

        # Create a ZMQ context
        self.context = zmq.Context()
        # Create a ZMQ subscriber
        self.subscriber = self.context.socket(zmq.SUB)
        # Set high water mark
        self.subscriber.set_hwm(hwm)
        # Set conflate
        self.subscriber.setsockopt(zmq.CONFLATE, conflate)
        # Connect the address
        self.subscriber.connect(f"tcp://{host}:{port}")
        # Subscribe all messages
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "")

        # Init the message
        self.magiclaw = magiclaw_msg_pb2.MagiClaw()

        print("Package Claw")
        print("Message Motor")
        print(
            "{\n\tfloat angle = 1;\n\tfloat speed = 2;\n\tfloat iq = 3;\n\tint32 temperature = 4;\n}"
        )
        print("Message Claw")
        print("{\n\tfloat angle = 1;\n\tMotor motor = 2;\n}")
        print("Message Finger")
        print(
            "{\n\tbytes img = 1;\n\trepeated float pose = 2;\n\trepeated float force = 3;\n\trepeated float node = 4;\n}"
        )
        print("Message Phone")
        print(
            "{\n\tbytes color_img = 1;\n\trepeated int32 depth_img = 2\n\trepeated int32 depth_width = 3\n\trepeated int32 depth_height = 4\n}"
        )
        print("Message MagiClaw")
        print(
            "{\n\tfloat timestamp = 1;\n\tClaw claw = 2;\n\tFinger finger_0 = 3;\n\tFinger finger_1 = 4;\n\tPhone phone = 5;\n}"
        )

        print("Claw Subscriber Initialization Done.")
        print("{:-^80}".format(""))

    def subscribeMessage(
        self,
    ) -> Tuple[
        float,
        float,
        float,
        int,
        bytes,
        np.ndarray,
        np.ndarray,
        np.ndarray,
        bytes,
        np.ndarray,
        np.ndarray,
        np.ndarray,
        bytes,
        np.ndarray,
        np.ndarray,
    ]:
        """Subscribe the message.

        Returns:
            The message.
        """

        # Receive the message
        self.magiclaw.ParseFromString(self.subscriber.recv())

        # Unpack the message
        claw_angle = self.magiclaw.claw.angle
        motor_angle = self.magiclaw.claw.motor.angle
        motor_speed = self.magiclaw.claw.motor.speed
        motor_iq = self.magiclaw.claw.motor.iq
        motor_temperature = self.magiclaw.claw.motor.temperature
        finger_0_img = self.magiclaw.finger_0.img
        finger_0_pose = np.array(self.magiclaw.finger_0.pose)
        finger_0_force = np.array(self.magiclaw.finger_0.force)
        finger_0_node = np.array(self.magiclaw.finger_0.node).reshape(-1, 3)
        finger_1_img = self.magiclaw.finger_1.img
        finger_1_pose = np.array(self.magiclaw.finger_1.pose)
        finger_1_force = np.array(self.magiclaw.finger_1.force)
        finger_1_node = np.array(self.magiclaw.finger_1.node).reshape(-1, 3)
        phone_color_img = self.magiclaw.phone.color_img
        phone_depth_img = self.magiclaw.phone.depth_img
        phone_local_pose = np.array(self.magiclaw.phone.local_pose)
        phone_global_pose = np.array(self.magiclaw.phone.global_pose)

        return (
            claw_angle,
            motor_angle,
            motor_speed,
            motor_iq,
            motor_temperature,
            finger_0_img,
            finger_0_pose,
            finger_0_force,
            finger_0_node,
            finger_1_img,
            finger_1_pose,
            finger_1_force,
            finger_1_node,
            phone_color_img,
            phone_depth_img,
            phone_local_pose,
            phone_global_pose,
        )

    def close(self):
        """Close ZMQ socket and context to prevent memory leaks."""
        if hasattr(self, "subscriber") and self.subscriber:
            self.subscriber.close()
        if hasattr(self, "context") and self.context:
            self.context.term()
