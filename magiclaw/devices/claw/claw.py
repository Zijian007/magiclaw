#!/usr/bin/env python

"""
Claw
====

The claw is one of the main components of the MagiClaw.
It is used to grasp and manipulate objects in various tasks.

Usage
-----

This module provides a class to control the claw. To initialize the claw, you can use the following code:

```
from magiclaw.devices.claw import Claw

claw = Claw(
    claw_id=<claw_id>,
    lead=<lead>,
    gear_radius=<gear_radius>,
    **motor_params
)
```

where `<claw_id>` is the ID of the claw, `<lead>` is the lead of the screw, `<gear_radius>` is the radius of the gear,
and `motor_params` are the parameters of the motor.

The claw class provides methods to control the claw, including opening and closing the claw, position control,
torque control, spring damping control, bilateral control, and bilateral spring damping control.

Usually, when the claw is used standalone, it can be controlled by the `spring_damping_control` method:

```
claw.spring_damping_control(target_angle=0.0)
```

It will work like a spring damper, automatically balancing at the target angle when there is no external force.

When the claw is used in teleoperation, it can be controlled by the `bilateral_spring_damping_control` method:

```
claw.bilateral_spring_damping_control(
    bilateral_motor_angle_percent=0.5,
    bilateral_motor_speed=0.0,
    target_angle=0.0
)
```

where `bilateral_motor_angle_percent` is the percentage of the motor angle, `bilateral_motor_speed` is the speed of
the motor, and `target_angle` is the target angle of the claw. The `mode` parameter can be set to "leader" or "follower".
It will keep two claws in the same angle and speed, given force feedback for users.
"""

import argparse
import sys
import time
import yaml
import numpy as np
from pylkmotor import LKMotor


class Claw:
    """
    Claw class.

    This class is used to control the claw.
    """

    def __init__(
        self,
        claw_id: int,
        bus_interface: str,
        bus_channel: str,
        motor_id: int,
        Kp_b: float,
        Kp_s: float,
        Kd: float,
        iq_max: float,
        angle_range: float,
        lead: float,
        gear_radius: float,
        motor_angle_deadband: int = 10,
        motor_speed_deadband: int = 10,
        mode: str = None,
    ) -> None:
        """Claw initialization.

        Args:
            claw_id (int): The ID of the claw.
            bus_interface (str): The bus interface.
            bus_channel (str): The bus channel.
            motor_id (int): The motor ID.
            Kp (float): The proportional gain of the PID controller.
            Kd (float): The derivative gain of the PID controller.
            iq_max (float): The maximum current.
            angle_range (float): The motor angle range in degrees.
            lead (float): The lead of the screw.
            gear_radius (float): The radius of the gear.
            motor_angle_deadband (int): The deadband of the motor angle in 0.01 degrees.
            motor_speed_deadband (int): The deadband of the motor speed in dps.
            mode (str): The mode of the claw, which can be "leader" or "follower".
        Raises:
            ValueError: If the bus interface is not valid or the motor ID is not valid.
        """

        # Set claw parameters
        self.claw_id = claw_id
        self.lead = lead
        self.gear_radius = gear_radius
        if mode not in ["leader", "follower"]:
            raise ValueError(
                f"Invalid mode: {mode}. The mode should be 'leader' or 'follower'."
            )
        self.mode = mode
        self.claw_angle = 0

        # Set motor parameters
        self.bus_interface = bus_interface
        self.bus_channel = bus_channel
        self.motor_id = motor_id
        self.Kp_b = Kp_b
        self.Kp_s = Kp_s
        self.Kd = Kd
        self.iq_max = iq_max
        self.motor_angle_offset = 0
        self.motor_angle = 0
        self.motor_angle_range = angle_range
        self.motor_angle_percent = 0
        self.motor_speed = 0
        self.motor_iq = 0
        self.motor_temperature = 0
        self.motor_angle_deadband = motor_angle_deadband
        self.motor_speed_deadband = motor_speed_deadband

        # Create a LKMotor object
        try:
            self.motor = LKMotor(self.bus_interface, self.bus_channel, self.motor_id)
        except Exception as e:
            raise ValueError(
                f"Failed to create a motor object: {e}. Please check the connection."
            )

        # Print the initialization message
        print("{:-^80}".format(f" Claw {self.claw_id} Initialization "))
        print(f"Bustype: {self.bus_interface}")
        print(f"Channel: {self.bus_channel}")
        print(f"Motor ID: {self.motor_id}")
        print(f"Kp_b: {self.Kp_b}")
        print(f"Kp_s: {self.Kp_s}")
        print(f"Kd: {self.Kd}")
        print(f"Max IQ: {self.iq_max} A")

        # Set motor to the initial position
        self.init()
        print("Claw Init.")
        print("Motor Angle Offset: ", self.motor_angle_offset)
        print("Motor Angle Range: ", self.motor_angle_range)

        # Read motor status
        self.read_motor_status()
        print(
            f"Motor Angle: {self.motor_angle} deg\nMotor Speed: {self.motor_speed} dps\nMotor IQ: {self.motor_iq} A"
        )

        print("Claw Initialization Done.")
        print("{:-^80}".format(""))

    def init(self) -> None:
        """
        Reset the motor to the initial position.

        The motor counterclockwise rotates to the minimum
        angle which is recorded as the minimum of the angle.
        The offset is calculated to make the minimum angle to be 0.
        """

        # Set the motor to the minimum angle (open)
        self.open()
        time.sleep(1)
        self.motor_angle_offset = self.motor.read_multi_turn_angle() / 100.0

        # Set the motor to the initial position
        self.motor.multi_turn_position_control(
            int(self.motor_angle_offset * 100.0), 1000
        )

    def open(self) -> None:
        """
        Open the motor.
        """

        self.motor.torque_loop_control(-100)

    def close(self) -> None:
        """
        Close the motor.
        """

        self.motor.torque_loop_control(100)

    def stop(self) -> None:
        """
        Stop the motor.
        """

        self.motor.motor_stop()

    def release(self) -> None:
        """
        Release the motor.
        """

        self.motor.bus.shutdown()

    def _motor_angle_to_claw_angle(self, angle: float) -> float:
        """
        Convert the motor angle to the claw angle.

        Args:
            angle (float): The motor angle in degrees.

        Returns:
            claw_angle (float): The claw angle in degrees.
        """

        # Convert the motor angle to the lead distance
        lead_distance = (self.motor_angle_range - angle) / 360.0 * self.lead
        # Convert the lead distance to the claw angle
        claw_angle = lead_distance / (2 * np.pi * self.gear_radius) * 360.0
        return claw_angle

    def _claw_angle_to_motor_angle(self, angle: float) -> float:
        """
        Convert the claw angle to the motor angle.

        Args:
            angle (float): The claw angle in degrees.

        Returns:
            motor_angle (float): The motor angle in degrees.
        """

        # Convert the claw angle to the lead distance
        lead_distance = angle / 360.0 * (2 * np.pi * self.gear_radius)
        # Convert the lead distance to the motor angle
        motor_angle = self.motor_angle_range - lead_distance / self.lead * 360.0
        return motor_angle

    def read_motor_status(self) -> None:
        """
        Get the motor status.

        This function reads and updtes the motor angle, temperature, iq, and speed.

        Raises:
            ValueError: If the motor status cannot be read.
        """

        try:
            # Read the motor angle
            motor_angle = self.motor.read_multi_turn_angle()
            self.motor_angle = motor_angle / 100.0 - self.motor_angle_offset
            self.claw_angle = self._motor_angle_to_claw_angle(self.motor_angle)
            self.motor_angle_percent = self.motor_angle / self.motor_angle_range

            # Read the motor temperature, iq, speed
            motor_temperature, motor_iq, motor_speed, _ = (
                self.motor.read_motor_status_2()
            )
            self.motor_temperature = motor_temperature
            self.motor_speed = motor_speed
            self.motor_iq = motor_iq / 2048 * 16.5
        except Exception as e:
            raise ValueError(f"Failed to read the motor status: {e}")

    def position_control(self, angle: float, speed: int = 1000) -> None:
        """
        Position control.

        This function sets the target position and speed of the motor.

        Args:
            angle (float): The target claw angle in degrees.
            speed (int): The speed of the motor in dps.
        """

        # Convert the claw angle to the motor angle
        motor_angle = self._claw_angle_to_motor_angle(angle)
        # Set the target position
        self.motor.multi_turn_position_control(
            int((motor_angle + self.motor_angle_offset) * 100.0), speed
        )

    def torque_control(self, iq: float) -> None:
        """Torque control.

        This function sets the target current of the motor.

        Args:
            iq (float): The target iq in A.
        """

        # Set the target current
        self.motor.torque_loop_control(int(iq * 2048 / 16.5))

    def spring_damping_control(self, target_angle: float = 0.0) -> None:
        """
        Spring damping control.

        This method is used to control the motor to automatically
        balance at the target angle when there is external force.
        Kp is the proportional gain and Kd is the derivative gain.
        The target current is calculated by the following formula:
        iq = -Kp_s * error - Kd * speed
        and then limited by the maximum current.

        Args:
            target_angle (float): The target motor angle in degrees.
        """

        # Read the motor status
        self.read_motor_status()

        # Calculate the error
        error = (self.motor_angle - target_angle) * 100.0

        # Calculate the target current
        iq = -self.Kp_s * error - self.Kd * self.motor_speed

        # Limit the target current
        iq = max(min(iq, self.iq_max), -self.iq_max)

        # Set the target current
        self.torque_control(iq)

    def bilateral_control(
        self, bilateral_angle: float = 0.0, bilateral_speed: float = 0.0
    ) -> None:
        """
        Bilateral control.

        This method is used to control the two motors keeping
        the same angle and speed. The target current is calculated
        by the following formula:
        iq = -Kp_b * angle_error - Kd * speed_error
        and then limited by the maximum current.

        Args:
            bilateral_angle (float): The bilateral angle in degrees.
            bilateral_speed (float): The bilateral speed in dps.
        """

        # Read the motor status
        self.read_motor_status()

        # Calculate the error
        angle_error = (self.motor_angle - bilateral_angle) * 100.0
        speed_error = self.motor_speed - bilateral_speed

        # Apply deadband
        if abs(angle_error) < self.motor_angle_deadband:
            angle_error = 0
        if abs(speed_error) < self.motor_speed_deadband:
            speed_error = 0

        # Calculate the target current
        iq = -self.Kp_b * angle_error - self.Kd * speed_error

        # Limit the target current
        iq = max(min(iq, self.iq_max), -self.iq_max)

        # Set the target current
        self.torque_control(iq)

    def bilateral_spring_damping_control(
        self,
        bilateral_motor_angle_percent,
        bilateral_motor_speed,
        target_angle: float = 0.0,
    ) -> None:
        """
        Bilateral spring damping control.

        This method is used to control the two motors keeping
        the same angle and speed, and also automatically balance
        at the target angle when there is external force. The
        target current is calculated by the following formula:
        iq = - Kp_b * biliteral_angle_error - Kd * biliteral_speed_error - Kp_s * target_angle_error
        and then limited by the maximum current.

        Args:
            bilateral_angle (float): The bilateral angle in degrees.
            bilateral_speed (float): The bilateral speed in dps.
            target_angle (float): The target motor angle in degrees.
        """

        # Read the motor status
        self.read_motor_status()

        # Check if bilateral angle and speed are valid
        if bilateral_motor_angle_percent is None or bilateral_motor_speed is None:
            bilateral_angle_error = 0
            bilateral_speed_error = 0
        else:
            # Convert the bilateral angle percent to the motor angle
            bilateral_motor_angle = (
                self.motor_angle_range * bilateral_motor_angle_percent
            )
            # Calculate the error
            bilateral_angle_error = (self.motor_angle - bilateral_motor_angle) * 100.0
            bilateral_speed_error = self.motor_speed - bilateral_motor_speed
            # Apply deadband
            if abs(bilateral_angle_error) < self.motor_angle_deadband:
                bilateral_angle_error = 0
            if abs(bilateral_speed_error) < self.motor_speed_deadband:
                bilateral_speed_error = 0

        if self.mode == "leader":
            # Spring damping control
            target_angle_error = (self.motor_angle - target_angle) * 100.0
        elif self.mode == "follower":
            target_angle_error = 0

        # Calculate the target current
        iq = (
            -self.Kp_b * bilateral_angle_error
            - self.Kd * bilateral_speed_error
            - self.Kp_s * target_angle_error
        )

        # Limit the target current
        iq = max(min(iq, self.iq_max), -self.iq_max)

        # Set the target current
        self.torque_control(iq)


if __name__ == "__main__":
    # Parse the arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--claw_id", type=int, default=0, help="The ID of the claw.")
    args = parser.parse_args()

    # Load the claw parameters
    with open(f"./configs/claw/claw_{args.claw_id}.yaml", "r") as f:
        claw_params = yaml.load(f, Loader=yaml.Loader)

    # Create a Claw object
    claw = Claw(
        claw_id=claw_params["id"],
        lead=claw_params["lead"],
        gear_radius=claw_params["gear_radius"],
        **claw_params["motor"],
    )

    # Release the motor
    claw.stop()
