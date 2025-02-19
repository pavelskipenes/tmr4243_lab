#!/usr/bin/env python3

import sensor_msgs.msg
import numpy as np
from template_joystick_control.joystick_mapping import JoystickAxes, JoystickButtons


def joystick_simple(
        joystick: sensor_msgs.msg.Joy,
        buttons: JoystickButtons,
        axes: JoystickAxes) -> np.ndarray:
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

    tau = [
        joystick.axes[axes.LEFT_X],
        joystick.axes[axes.LEFT_Y],
        joystick.axes[axes.TRIGGER_LEFT],
        joystick.axes[axes.RIGHT_X],
        joystick.axes[axes.RIGHT_Y],
        joystick.axes[axes.TRIGGER_RIGHT],
    ]

    return np.array([tau], dtype=float).T
