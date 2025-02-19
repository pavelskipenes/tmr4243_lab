#!/usr/bin/env python3

import sensor_msgs.msg
import numpy as np
from template_joystick_control.joystick_mapping import JoystickAxes, JoystickButtons


def joystick_force_body_relative(
        joystick: sensor_msgs.msg.Joy,
        buttons: JoystickButtons,
        axes: JoystickAxes) -> np.ndarray:

    # Replace the following line
    u0, u1, u2, a1, a2 = 0.0, 0.0, 0.0, 0.0, 0.0

    #
    # Write your code below
    #
    u = np.array([[u0, u1, u2, a1, a2]], dtype=float).T
    return u

