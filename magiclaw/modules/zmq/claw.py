#!/usr/bin/env python

import zmq
from typing import Tuple
from datetime import datetime
from magiclaw.modules.protobuf import claw_msg_pb2


class ClawPublisher:
    def __init__(self, host: str = "0.0.0.0", port: int = None, hwm: int = 1, conflate: bool = True) -> None:
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
        self.claw = claw_msg_pb2.Claw()

        print("Package Claw")
        print("Message Claw")
        print(
            "{\n\tdouble timestamp = 1;\n\tfloat angle = 2;\n\tfloat speed = 3;\n\tfloat iq = 4;\n\tfloat temperature = 5;\n}"
        )

        print("Claw Publisher Initialization Done.")
        print("{:-^80}".format(""))

    def publishMessage(
        self,
        claw_angle: float = 0.0,
        motor_angle: float = 0.0,
        motor_angle_percent: float = 0.0,
        motor_speed: float = 0.0,
        motor_iq: float = 0.0,
        motor_temperature: int = 0,
    ) -> None:
        """Publish the message.

        Args:
            img: The image captured by the camera.
            pose: The pose of the marker.
            force: The force on the bottom surface of the finger.
            node: The node displacement of the finger.
        """

        # Set the message
        self.claw.timestamp = datetime.now().timestamp()
        self.claw.angle = claw_angle
        self.claw.motor.angle = motor_angle
        self.claw.motor.angle_percent = motor_angle_percent
        self.claw.motor.speed = motor_speed
        self.claw.motor.iq = motor_iq
        self.claw.motor.temperature = motor_temperature

        # Publish the message
        self.publisher.send(self.claw.SerializeToString())

    def close(self):
        """Close ZMQ socket and context to prevent memory leaks."""
        if hasattr(self, "publisher") and self.publisher:
            self.publisher.close()
        if hasattr(self, "context") and self.context:
            self.context.term()


class ClawSubscriber:
    def __init__(self, host, port, hwm: int = 1, conflate: bool = True, timeout: int = 100) -> None:
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
        # Subscribe the topic
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        # Set poller
        self.poller = zmq.Poller()
        self.poller.register(self.subscriber, zmq.POLLIN)
        self.socks = dict(self.poller.poll(timeout))

        # Init the message
        self.claw = claw_msg_pb2.Claw()

        print("Package Claw")
        print("Message Claw")
        print(
            "{\n\tdouble timestamp = 1;\n\tfloat angle = 2;\n\tfloat speed = 3;\n\tfloat iq = 4;\n\tfloat temperature = 5;\n}"
        )

        print("Claw Subscriber Initialization Done.")
        print("{:-^80}".format(""))

    def subscribeMessage(self) -> Tuple[float, float, float, int]:
        """Subscribe the message.

        Args:
            timeout: Maximum time to wait for a message in milliseconds. Default is 100ms.

        Returns:
            angle, speed, iq, temperature: The claw status data.

        Raises:
            zmq.ZMQError: If no message is received within the timeout period.
        """

        # Receive the message
        
        if self.subscriber in self.socks and self.socks[self.subscriber] == zmq.POLLIN:
            self.claw.ParseFromString(self.subscriber.recv())
        else:
            raise zmq.ZMQError("No message received within the timeout period.")
        return (
            self.claw.angle,
            self.claw.motor.angle,
            self.claw.motor.angle_percent,
            self.claw.motor.speed,
            self.claw.motor.iq,
            self.claw.motor.temperature,
        )

    def close(self):
        """Close ZMQ socket and context to prevent memory leaks."""
        if hasattr(self, "subscriber") and self.subscriber:
            self.subscriber.close()
        if hasattr(self, "context") and self.context:
            self.context.term()
