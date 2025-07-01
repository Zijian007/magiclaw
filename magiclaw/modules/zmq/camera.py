#!/usr/bin/env python

import re
import zmq
import pathlib
import numpy as np
from magiclaw.modules.protobuf import cam_msg_pb2
# import cam_msg_pb2  # Assuming cam_msg_pb2 is generated from cam_msg.proto

class CameraSubscriber:
    def __init__(
        self,
        host: str,
        port: int,
        hwm: int = 1,
        conflate: bool = True,
        timeout: int = 1000,
    ) -> None:
        """Subscriber initialization.

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
        with open(
            pathlib.Path(__file__).parent / "protobuf/camera_msg.proto",
        ) as f:
            lines = f.read()
        messages = re.search(
            r"message\s+Camera\s*{{(.*?)}}", lines, re.DOTALL
        )
        body = messages.group(1)
        print("message Camera")
        print("{\n" + body + "\n}")

        print("Camera Subscriber Initialization Done.")

    def subscribeMessage(self) -> np.ndarray:
        """Subscribe the message.

        Args:
            timeout: Maximum time to wait for a message in milliseconds. Default is 2000ms.

        Returns:
            img: The image captured by the camera.

        Raises:
            zmq.ZMQError: If no message is received within the timeout period.
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
        """Close ZMQ socket and context to prevent memory leaks."""
        if hasattr(self, "subscriber") and self.subscriber:
            try:
                self.poller.unregister(self.subscriber)
                self.subscriber.close()
                self.subscriber = None
            except Exception as e:
                print(f"Error closing subscriber: {e}")
        if hasattr(self, "context") and self.context:
            try:
                self.context.term()
                self.context = None
            except Exception as e:
                print(f"Error terminating context: {e}")
                
                
if __name__ == "__main__":
    # Example usage
    camera_subscriber = CameraSubscriber(host="192.168.4.202", port=5555)
    try:
        while True:
            img = camera_subscriber.subscribeMessage()
            print(f"Received image of shape: {img.shape}")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        camera_subscriber.close()
