#!/usr/bin/env python

import re
import zmq
import pathlib
import numpy as np
from typing import Tuple
from datetime import datetime
from magiclaw.modules.protobuf import finger_msg_pb2


class FingerPublisher:
    """
    FingerPublisher class.

    This class is used to publish Finger messages using ZeroMQ.

    Attributes:
        context (zmq.Context): The ZMQ context for the publisher.
        publisher (zmq.Socket): The ZMQ publisher socket.
    """

    def __init__(
        self,
        host: str,
        port: int,
        hwm: int = 1,
        conflate: bool = True,
    ) -> None:
        """
        Publisher initialization.

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

        # Read the protobuf definition for Finger message
        with open(
            pathlib.Path(__file__).parent.parent / "protobuf/finger_msg.proto",
        ) as f:
            lines = f.read()
        messages = re.search(r"message\s+Finger\s*{{(.*?)}}", lines, re.DOTALL)
        body = messages.group(1)
        print("Message Finger")
        print("{\n" + body + "\n}")

        print("Finger Publisher Initialization Done.")
        print("{:-^80}".format(""))

    def publishMessage(
        self,
        img_bytes: bytes = b"",
        pose: list = np.zeros(6, dtype=np.float32).tolist(),
        force: list = np.zeros(6, dtype=np.float32).tolist(),
        node: list = np.zeros(6, dtype=np.float32).tolist(),
    ) -> None:
        """
        Publish the message.

        Args:
            img_bytes (bytes): The image captured by the camera.
            pose (list): The pose of the marker.
            force (list): The force on the bottom surface of the finger.
            node (list): The node displacement of the finger.
        """

        # Set the message
        finger = finger_msg_pb2.Finger()
        finger.timestamp = datetime.now().timestamp()
        finger.img = img_bytes
        finger.pose[:] = pose
        finger.force[:] = force
        finger.node[:] = node

        # Publish the message
        self.publisher.send(finger.SerializeToString())

    def close(self):
        """
        Close ZMQ socket and context to prevent memory leaks.
        """
        
        if hasattr(self, "publisher") and self.publisher:
            self.publisher.close()
        if hasattr(self, "context") and self.context:
            self.context.term()


class FingerSubscriber:
    """
    FingerSubscriber class.

    This class subscribes to messages from a publisher and parses the received messages.

    Attributes:
        context (zmq.Context): The ZMQ context for the subscriber.
        subscriber (zmq.Socket): The ZMQ subscriber socket.
        poller (zmq.Poller): The ZMQ poller for handling timeouts.
        timeout (int): The maximum time to wait for a message in milliseconds.
    """

    def __init__(
        self,
        host: str,
        port: int,
        hwm: int = 1,
        conflate: bool = True,
        timeout: int = 100,
    ) -> None:
        """
        Subscriber initialization.

        Args:
            host (str): The host address of the subscriber.
            port (int): The port number of the subscriber.
            hwm (int): High water mark for the subscriber. Default is 1.
            conflate (bool): Whether to conflate messages. Default is True.
            timeout (int): Maximum time to wait for a message in milliseconds. Default is 100 ms.
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
        self.timeout = timeout

        # Read the protobuf definition for Finger message
        with open(
            pathlib.Path(__file__).parent.parent / "protobuf/finger_msg.proto",
        ) as f:
            lines = f.read()
        messages = re.search(r"message\s+Finger\s*{{(.*?)}}", lines, re.DOTALL)
        body = messages.group(1)
        print("Message Finger")
        print("{\n" + body + "\n}")

        print("Finger Subscriber Initialization Done.")
        print("{:-^80}".format(""))

    def subscribeMessage(self) -> Tuple[bytes, list, list, list]:
        """
        Subscribe the message.

        Returns:
            img_bytes (bytes): The image captured by the camera.
            pose (list): The pose of the marker.
            force (list): The force on the bottom surface of the finger.
            node (list): The node displacement of the finger.

        Raises:
            RuntimeError: If no message is received within the timeout period.
        """

        # Receive the message

        if self.poller.poll(self.timeout):
            # Receive the message
            msg = self.subscriber.recv()

            # Parse the message
            finger = finger_msg_pb2.Finger()
            finger.ParseFromString(msg)
        else:
            raise RuntimeError("No message received within the timeout period.")
        return (
            finger.img,
            finger.pose,
            finger.force,
            finger.node,
        )

    def close(self):
        """
        Close ZMQ socket and context to prevent memory leaks.
        """
        
        if hasattr(self, "subscriber") and self.subscriber:
            self.subscriber.close()
        if hasattr(self, "context") and self.context:
            self.context.term()
