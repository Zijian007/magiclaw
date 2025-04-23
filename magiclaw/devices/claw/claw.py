#!/usr/bin/env python

import argparse
import sys
import time
import yaml
import numpy as np
import copy
from pylkmotor import LKMotor


class Claw:
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
        angle_max: int,
        lead: float,
        gear_radius: float,
        angle_deadband: int = 10,
        speed_deadband: int = 10,
        motor_mode: str = None,
    ) -> None:
        """Claw initialization.

        Args:
            bus_interface: The bus interface.
            bus_channel: The bus channel.
            motor_id: The motor ID.
            Kp: The proportional gain of the PID controller.
            Kd: The derivative gain of the PID controller.
            iq_max: The maximum current.
        """

        # Set the parameters
        self.claw_id = claw_id
        self.bus_interface = bus_interface
        self.bus_channel = bus_channel
        self.motor_id = motor_id
        self.Kp_b = Kp_b
        self.Kp_s = Kp_s
        self.Kd = Kd
        self.iq_max = iq_max
        self.lead = lead
        self.gear_radius = gear_radius

        # Create a LKMotor object
        try:
            self.motor = LKMotor(self.bus_interface, self.bus_channel, self.motor_id)
        except Exception as e:
            print("\033[31mFailed to connect to the motor!\033[0m")
            print("\033[31mPlease check the connection!\033[0m")
            sys.exit()

        # Print the initialization message
        print("{:-^80}".format(f" Claw {self.claw_id} Initialization "))
        print(f"Bustype: {self.bus_interface}")
        print(f"Channel: {self.bus_channel}")
        print(f"Motor ID: {self.motor_id}")
        print(f"Kp_b: {self.Kp_b}")
        print(f"Kp_s: {self.Kp_s}")
        print(f"Kd: {self.Kd}")
        print(f"Max IQ: {self.iq_max} A")

        # Set parameters
        self.motor_angle_offset = 0
        self.motor_angle = 0
        self.motor_angle_min = 0
        self.motor_angle_max = angle_max
        self.motor_speed = 0
        self.motor_iq = 0
        self.motor_temperature = 0
        self.claw_angle = 0
        self.angle_deadband = angle_deadband
        self.speed_deadband = speed_deadband
        self.motor_mode = motor_mode

        # Read motor status
        self.read_motor_status()

        # Set motor to the initial position
        self.init()
        print("Claw Init.")
        print("Motor Angle Offset: ", self.motor_angle_offset)
        print("Motor Angle Min: ", self.motor_angle_min)
        print("Motor Angle Max: ", self.motor_angle_max)

        print("Claw Initialization Done.")
        print("{:-^80}".format(""))

    def init(self) -> None:
        """Reset the motor to the initial position.

        The motor counterclockwise rotates to the minimum
        angle which is recorded as the minimum of the angle.
        The offset is calculated to make the minimum angle to be 0.

        """

        # Set the motor to the minimum angle (open)
        self.open()
        time.sleep(1)
        motor_angle_min = self.motor.read_multi_turn_angle()

        # Calculate the offset
        self.motor_angle_offset = motor_angle_min / 100
        self.motor_angle_min = 0

        # Set the motor to the initial position
        self.motor.multi_turn_position_control(int(self.motor_angle_offset * 100), 1000)

    def open(self) -> None:
        """Open the motor."""

        self.motor.torque_loop_control(-150)

    def close(self) -> None:
        """Close the motor."""

        self.motor.torque_loop_control(150)

    def stop(self) -> None:
        """Stop the motor."""

        self.motor.motor_stop()

    def release(self) -> None:
        """Release the motor."""

        self.motor.bus.shutdown()

    def _motor_angle_to_claw_angle(self, angle: float) -> float:
        """Convert the motor angle to the claw angle.

        Args:
            angle: The motor angle in degrees.

        Returns:
            claw_angle: The claw angle in degrees.
        """

        # Convert the motor angle to the lead distance
        lead_distance = (self.motor_angle_max - angle) / 360.0 * self.lead
        # Convert the lead distance to the claw angle
        claw_angle = lead_distance / (2 * np.pi * self.gear_radius) * 360.0
        return claw_angle

    def _claw_angle_to_motor_angle(self, angle: float) -> float:
        """Convert the claw angle to the motor angle.

        Args:
            angle: The claw angle in degrees.

        Returns:
            motor_angle: The motor angle in degrees.
        """

        # Convert the claw angle to the lead distance
        lead_distance = angle / 360.0 * (2 * np.pi * self.gear_radius)
        # Convert the lead distance to the motor angle
        motor_angle = self.motor_angle_max - lead_distance / self.lead * 360.0
        return motor_angle

    def read_motor_status(self):
        """Get the motor status.

        Returns:
            motor_angle: The motor angle in 0.01 degrees.
            motor_speed: The motor speed in dps.
            motor_iq: The motor iq in A.
        """

        try:
            # Read the motor angle
            motor_angle = self.motor.read_multi_turn_angle()
            self.motor_angle = motor_angle / 100 - self.motor_angle_offset
            self.claw_angle = self._motor_angle_to_claw_angle(self.motor_angle)

            # Read the motor temperature, iq, speed
            motor_temperature, motor_iq, motor_speed, _ = (
                self.motor.read_motor_status_2()
            )
            self.motor_temperature = motor_temperature
            self.motor_speed = motor_speed
            self.motor_iq = motor_iq / 2048 * 16.5
        except TypeError:
            print("\033[31mFailed to read motor status!\033[0m")
            print("\033[31mPlease check the connection!\033[0m")
            sys.exit()

    def position_control(self, angle: float, speed: int = 1000) -> None:
        """Position control.

        Args:
            angle: The target claw angle in degrees.
            speed: The speed of the motor in dps.
        """

        # Convert the claw angle to the motor angle
        motor_angle = self._claw_angle_to_motor_angle(angle)
        # Set the target position
        self.motor.multi_turn_position_control(
            int((motor_angle + self.motor_angle_offset) * 100), speed
        )

    def torque_control(self, iq: float) -> None:
        """Torque control.

        Args:
            iq: The target iq in A.
        """

        # Set the target current
        self.motor.torque_loop_control(int(iq * 2048 / 16.5))

    def spring_damping_control(self, target_angle: float = 0.0) -> None:
        """Spring damping control.

        This method is used to control the motor to automatically
        balance at the target angle when there is external force.
        Kp is the proportional gain and Kd is the derivative gain.
        The target current is calculated by the following formula:
        iq = -Kp_s * error - Kd * speed
        and then limited by the maximum current.

        Args:
            angle: The target motor angle in degrees.
            speed: The speed of the motor in dps.
        """

        # Read the motor status
        self.read_motor_status()

        # Calculate the error
        error = (self.motor_angle - target_angle) * 100

        # Calculate the target current
        iq = -self.Kp_s * error - self.Kd * self.motor_speed

        # Limit the target current
        iq = max(min(iq, self.iq_max), -self.iq_max)

        # Set the target current
        self.torque_control(iq)

    def bilateral_control(
        self, bilateral_angle: float = 0.0, bilateral_speed: float = 0.0
    ) -> None:
        """Bilateral control.

        This method is used to control the two motors keeping
        the same angle and speed. The target current is calculated
        by the following formula:
        iq = -Kp_b * angle_error - Kd * speed_error
        and then limited by the maximum current.

        Args:
            bilateral_angle: The target bilateral angle in degrees.
            bilateral_speed: The target bilateral speed in dps.
            motor_angle: The target motor angle in degrees.
        """

        # Read the motor status
        self.read_motor_status()

        # Calculate the error
        angle_error = (self.motor_angle - bilateral_angle) * 100
        speed_error = self.motor_speed - bilateral_speed

        # Apply deadband
        if abs(angle_error) < self.angle_deadband:
            angle_error = 0
        if abs(speed_error) < self.speed_deadband:
            speed_error = 0

        # Calculate the target current
        iq = -self.Kp_b * angle_error - self.Kd * speed_error

        # Limit the target current
        iq = max(min(iq, self.iq_max), -self.iq_max)

        # Set the target current
        self.torque_control(iq)

    def bilateral_spring_damping_control(
        self,
        bilateral_angle,
        bilateral_speed,
        target_angle: float = 0.0,
    ) -> None:
        """Bilateral spring damping control.

        This method is used to control the two motors keeping
        the same angle and speed, and also automatically balance
        at the target angle when there is external force. The
        target current is calculated by the following formula:
        iq = - Kp_b * biliteral_angle_error - Kd * biliteral_speed_error - Kp_s * target_angle_error
        and then limited by the maximum current.

        Args:
            bilateral_angle: The target bilateral angle in degrees.
            bilateral_speed: The target bilateral speed in dps.
            target_angle: The target motor angle in degrees.
        """

        # Read the motor status
        self.read_motor_status()
        
        # Check if bilateral angle and speed are valid
        if bilateral_angle is None or bilateral_speed is None:
            bilateral_angle_error = 0
            bilateral_speed_error = 0
        else:
            # Calculate the error
            bilateral_angle_error = (self.motor_angle - bilateral_angle) * 100
            bilateral_speed_error = self.motor_speed - bilateral_speed
            # Apply deadband
            if abs(bilateral_angle_error) < self.angle_deadband:
                bilateral_angle_error = 0
            if abs(bilateral_speed_error) < self.speed_deadband:
                bilateral_speed_error = 0
        
        target_angle_error = (self.motor_angle - target_angle) * 100

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
    parser.add_argument(
        "--spring_damping_control",
        action="store_true",
        help="Demo spring damping control to balance the motor.",
    )
    args = parser.parse_args()

    # Load the claw parameters
    with open(f"./config/claw/claw_{args.claw_id}.yaml", "r") as f:
        claw_params = yaml.load(f, Loader=yaml.Loader)

    # Create a Claw object
    claw = Claw(
        claw_id=claw_params["id"],
        lead=claw_params["lead"],
        gear_radius=claw_params["gear_radius"],
        **claw_params["motor"],
    )

    # Demo spring damping control
    if args.spring_damping_control:
        print("Spring damping control ...")
        try:
            while True:
                claw.spring_damping_control()
                time.sleep(0.01)
                print(
                    f"Angle: {claw.motor_angle} deg, Speed: {claw.motor_speed} dps, IQ: {claw.motor_iq} A"
                )
        except KeyboardInterrupt:
            claw.stop()
            print("\nClaw Stopped.")
