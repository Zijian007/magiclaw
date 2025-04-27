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

from .claw import Claw