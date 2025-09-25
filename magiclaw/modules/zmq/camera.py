#!/usr/bin/env python

import re
import zmq
import pathlib
import numpy as np
from magiclaw.modules.protobuf import cam_msg_pb2


class CameraSubscriber:
    """
    CameraSubscriber class.

    This class is used to subscribe to camera messages using ZeroMQ.

    Attributes:
        context (zmq.Context): The ZeroMQ context.
        subscriber (zmq.Socket): The ZeroMQ subscriber socket.
        poller (zmq.Poller): The ZeroMQ poller for handling timeouts.
        timeout (int): The maximum time to wait for a message in milliseconds.
    """

    def __init__(
        self,
        host: str,
        port: int,
        hwm: int = 1,
        conflate: bool = True,
        timeout: int = 1000,
    ) -> None:
        """
        Subscriber initialization.

        Args:
            host (str): The host address of the subscriber.
            port (int): The port number of the subscriber.
            hwm (int): High water mark for the subscriber. Default is 1.
            conflate (bool): Whether to conflate messages. Default is True.
            timeout (int): Maximum time to wait for a message in milliseconds. Default is 1000 ms.
        """

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
        # Use poller to implement timeout
        self.poller = zmq.Poller()
        self.poller.register(self.subscriber, zmq.POLLIN)
        self.timeout = timeout

        # Read the protobuf definition for Camera message
        # with open(
        #     pathlib.Path(__file__).parent.parent / "protobuf/cam_msg.proto",
        # ) as f:
        #     lines = f.read()
        # messages = re.search(r"message\s+Camera\s*{{(.*?)}}", lines, re.DOTALL)
        # body = messages.group(1)
        # print("message Camera")
        # print("{\n" + body + "\n}")

        print("Camera Subscriber Initialization Done.")

    def subscribeMessage(self) -> np.ndarray:
        """
        Subscribe the message.

        Returns:
            img: The image captured by the camera.

        Raises:
            RuntimeError: If no message is received within the timeout period.
        """
        # Wait for message with timeout
        if self.poller.poll(self.timeout):
            # Receive the message
            msg = self.subscriber.recv()

            # Parse the message
            cam = cam_msg_pb2.Camera()
            cam.ParseFromString(msg)

            return np.frombuffer(cam.img, dtype=np.uint8)
        else:
            raise RuntimeError("No message received within the timeout period.")

    def close(self):
        """
        Close ZMQ socket and context.
        """
        
        if hasattr(self, "subscriber") and self.subscriber:
            self.subscriber.close()
        if hasattr(self, "context") and self.context:
            self.context.term()
