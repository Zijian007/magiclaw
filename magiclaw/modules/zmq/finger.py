#!/usr/bin/env python

import zmq
import numpy as np
from typing import Tuple
from datetime import datetime
from magiclaw.modules.protobuf import finger_msg_pb2


class FingerPublisher:
    def __init__(self, host, port, hwm: int = 1, conflate: bool = True) -> None:
        """Publisher initialization.

        Args:
            host (str): The host address of the publisher.
            port (int): The port number of the publisher.
            hwm (int): High water mark for the publisher. Default is 1.
            conflate (bool): Whether to conflate messages. Default is True.
        """

        print("{:-^80}".format(" Finger Publisher Initialization "))
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
        self.finger = finger_msg_pb2.Finger()

        print("Package Finger")
        print("Message Finger")
        print(
            "{\n\tbytes img = 1;\n\trepeated float pose = 2;\n\trepeated float force = 3;\n\trepeated float node = 4;\n}"
        )

        print("Finger Publisher Initialization Done.")
        print("{:-^80}".format(""))

    def publishMessage(
        self,
        img_bytes: bytes = b"",
        pose: list = np.zeros(6, dtype=np.float32).tolist(),
        force: list = np.zeros(6, dtype=np.float32).tolist(),
        node: list = np.zeros(6, dtype=np.float32).tolist(),
    ) -> None:
        """Publish the message.

        Args:
            img: The image captured by the camera.
            pose: The pose of the marker (numpy array or list).
            force: The force on the bottom surface of the finger (numpy array or list).
            node: The node displacement of the finger (numpy array or list).
        """

        # Set the message
        self.finger.timestamp = datetime.now().timestamp()
        self.finger.img = img_bytes
        self.finger.pose[:] = pose
        self.finger.force[:] = force
        self.finger.node[:] = node

        # Publish the message
        self.publisher.send(self.finger.SerializeToString())

    def close(self):
        """Close ZMQ socket and context to prevent memory leaks."""
        if hasattr(self, "publisher") and self.publisher:
            self.publisher.close()
        if hasattr(self, "context") and self.context:
            self.context.term()


class FingerSubscriber:
    def __init__(self, host, port, hwm: int = 1, conflate: bool = True, timeout: int = 100) -> None:
        """Subscriber initialization.

        Args:
            host (str): The host address of the subscriber.
            port (int): The port number of the subscriber.
            hwm (int): High water mark for the subscriber. Default is 1.
            conflate (bool): Whether to conflate messages. Default is True.
        """

        print("{:-^80}".format(" Finger Subscriber Initialization "))
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
        self.finger = finger_msg_pb2.Finger()

        print("Package Finger")
        print("Message Finger")
        print(
            "{\n\tbytes img = 1;\n\trepeated float pose = 2;\n\trepeated float force = 3;\n\trepeated float node = 4;\n}"
        )

        print("Finger Subscriber Initialization Done.")
        print("{:-^80}".format(""))

    def subscribeMessage(self) -> Tuple[bytes, np.ndarray, np.ndarray, np.ndarray]:
        """Subscribe the message.

        Args:
            timeout: Maximum time to wait for a message in milliseconds. Default is 100ms.

        Returns:
            img: The image captured by the camera.
            pose: The pose of the marker.
            force: The force on the bottom surface of the finger.
            node: The node displacement of the finger.

        Raises:
            zmq.ZMQError: If no message is received within the timeout period.
        """

        # Receive the message
        
        if self.subscriber in self.socks and self.socks[self.subscriber] == zmq.POLLIN:
            self.finger.ParseFromString(self.subscriber.recv())
        else:
            raise zmq.ZMQError("No message received within the timeout period.")
        return (
            self.finger.img,
            self.finger.pose,
            self.finger.force,
            self.finger.node,
        )

    def close(self):
        """Close ZMQ socket and context to prevent memory leaks."""
        if hasattr(self, "subscriber") and self.subscriber:
            self.subscriber.close()
        if hasattr(self, "context") and self.context:
            self.context.term()
