#!/usr/bin/env python

import zmq
import numpy as np
from typing import Tuple
from datetime import datetime
from magiclaw.modules.protobuf import phone_msg_pb2


class PhonePublisher:
    def __init__(self, host, port, hwm: int = 1, conflate: bool = True) -> None:
        """Publisher initialization.

        Args:
            host (str): The host address of the publisher.
            port (int): The port number of the publisher.
            hwm (int): High water mark for the publisher. Default is 1.
            conflate (bool): Whether to conflate messages. Default is True.
        """

        print("{:-^80}".format(" Phone Publisher Initialization "))
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
        self.phone = phone_msg_pb2.Phone()

        print("Package Phone")
        print("Message Phone")
        print(
            "{\n\tbytes img = 1;\n\trepeated float pose = 2;\n\trepeated float force = 3;\n\trepeated float node = 4;\n}"
        )

        print("Phone Publisher Initialization Done.")
        print("{:-^80}".format(""))

    def publishMessage(
        self,
        color_img_bytes: bytes = b"",
        depth_img_bytes: bytes = b"",
        depth_width: int = 256,
        depth_height: int = 192,
        local_pose: list = np.zeros(6, dtype=np.float32).tolist(),
        global_pose: list = np.zeros(6, dtype=np.float32).tolist(),
    ) -> None:
        """Publish the message.

        Args:
            color_img_bytes: The image captured by the camera.
            depth_img: The depth image captured by the camera.
            pose: The pose of the marker (numpy array or list).
        """

        # Set the message
        self.phone.timestamp = datetime.now().timestamp()
        self.phone.color_img = color_img_bytes
        self.phone.depth_img = depth_img_bytes
        self.phone.depth_width = depth_width
        self.phone.depth_height = depth_height
        self.phone.local_pose[:] = local_pose.flatten().tolist()
        self.phone.global_pose[:] = global_pose.flatten().tolist()

        # Publish the message
        self.publisher.send(self.phone.SerializeToString())

    def close(self):
        """Close ZMQ socket and context to prevent memory leaks."""
        if hasattr(self, "publisher") and self.publisher:
            self.publisher.close()
        if hasattr(self, "context") and self.context:
            self.context.term()


class PhoneSubscriber:
    def __init__(self, host, port, hwm: int = 1, conflate: bool = True, timeout: int = 100) -> None:
        """Subscriber initialization.

        Args:
            host (str): The host address of the subscriber.
            port (int): The port number of the subscriber.
            hwm (int): High water mark for the subscriber. Default is 1.
            conflate (bool): Whether to conflate messages. Default is True.
        """

        print("{:-^80}".format(" Phone Subscriber Initialization "))
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
        # Subscribe the topic
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        # Set poller
        self.poller = zmq.Poller()
        self.poller.register(self.subscriber, zmq.POLLIN)
        self.socks = dict(self.poller.poll(timeout))

        # Init the message
        self.phone = phone_msg_pb2.Phone()

        print("Package Phone")
        print("Message Phone")
        print(
            "{\n\tbytes img = 1;\n\trepeated float pose = 2;\n\trepeated float force = 3;\n\trepeated float node = 4;\n}"
        )

        print("Phone Subscriber Initialization Done.")
        print("{:-^80}".format(""))

    def subscribeMessage(self) -> Tuple[bytes, list, int, int, list, list]:
        """Subscribe the message.

        Args:
            timeout: Maximum time to wait for a message in milliseconds. Default is 100ms.

        Returns:
            color_img: The image captured by the camera.
            depth_img: The depth image captured by the camera.
            depth_width: The width of the depth image.
            depth_height: The height of the depth image.
            local_pose: The local pose of the phone.
            global_pose: The global pose of the phone.

        Raises:
            zmq.ZMQError: If no message is received within the timeout period.
        """

        # Receive the message
        
        if self.subscriber in self.socks and self.socks[self.subscriber] == zmq.POLLIN:
            msg = self.subscriber.recv()
            # Parse the message
            self.phone.ParseFromString(msg)
        else:
            raise zmq.ZMQError("No message received within the timeout period.")
        return (
            self.phone.color_img,
            self.phone.depth_img,
            self.phone.depth_width,
            self.phone.depth_height,
            self.phone.local_pose,
            self.phone.global_pose,
        )

    def close(self):
        """Close ZMQ socket and context to prevent memory leaks."""
        if hasattr(self, "subscriber") and self.subscriber:
            self.subscriber.close()
        if hasattr(self, "context") and self.context:
            self.context.term()
