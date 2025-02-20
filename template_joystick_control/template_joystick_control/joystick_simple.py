#!/usr/bin/env python3

import sensor_msgs.msg
import numpy as np
from template_joystick_control.joystick_mapping import JoystickAxes, JoystickButtons


def joystick_simple(
        joystick: sensor_msgs.msg.Joy,
        buttons: JoystickButtons,
        axes: JoystickAxes) -> np.ndarray:

    # Tunnel
    u0 = (joystick.axes[axes.TRIGGER_RIGHT] -
          joystick.axes[axes.TRIGGER_LEFT]) / 2.0

    # Starboard
    u1 = np.linalg.norm(np.array((
        joystick.axes[axes.LEFT_X], joystick.axes[axes.LEFT_Y]
    )))
    a1 = np.arctan2(joystick.axes[axes.LEFT_X], joystick.axes[axes.LEFT_Y])

    # Starboard
    u2 = np.linalg.norm(np.array((
        joystick.axes[axes.RIGHT_X], joystick.axes[axes.RIGHT_Y]
    )))
    a2 = np.arctan2(joystick.axes[axes.RIGHT_X], joystick.axes[axes.RIGHT_X])

    tau = [u0, u1, a1, u2, a2]

    return np.array(tau, dtype=float).T
