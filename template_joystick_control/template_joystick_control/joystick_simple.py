#!/usr/bin/env python3

import sensor_msgs.msg
import numpy as np
from template_joystick_control.joystick_mapping import JoystickMapping

def joystick_simple(
        joystick: sensor_msgs.msg.Joy,
        mapping: JoystickMapping) -> np.ndarray:
    # Replace the following line
    u0, u1, u2, a1, a2 = 0.0, 0.0, 0.0, 0.0, 0.0

    # # Tunnel
    # u0 = (joystick.axes[mapping.RIGHT_TRIGGER] - joystick.axes[mapping.LEFT_TRIGGER]) / 2.0
    #
    # # Starboard
    # u1 = np.linalg.norm(np.array((
    #     joystick.axes[mapping.LEFT_STICK_HORIZONTAL], joystick.axes[mapping.LEFT_STICK_VERTICAL]
    # )))
    # a1 = np.arctan2(joystick.axes[mapping.LEFT_STICK_HORIZONTAL], joystick.axes[mapping.LEFT_STICK_VERTICAL])
    #
    # # Starboard
    # u2 = np.linalg.norm(np.array((
    #     joystick.axes[mapping.RIGHT_STICK_HORIZONTAL], joystick.axes[mapping.RIGHT_STICK_VERTICAL]
    # )))
    # a2 = np.arctan2(joystick.axes[mapping.RIGHT_STICK_HORIZONTAL], joystick.axes[mapping.RIGHT_STICK_VERTICAL])
    #

    u0 = (joystick.axes[mapping.RIGHT_TRIGGER] - joystick.axes[mapping.LEFT_TRIGGER]) / 2.0
    u1 = joystick.axes[mapping.LEFT_STICK_HORIZONTAL]
    u2 = joystick.axes[mapping.LEFT_STICK_VERTICAL]
    a1 = joystick.axes[mapping.RIGHT_STICK_HORIZONTAL]
    a2 = joystick.axes[mapping.RIGHT_STICK_VERTICAL]

    u = np.array([[u0, u1, u2, a1, a2]], dtype=float).T
    return u
