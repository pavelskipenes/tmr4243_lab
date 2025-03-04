#!/usr/bin/env python3

import sensor_msgs.msg
import numpy as np

from typing import Tuple

from template_joystick_control.joystick_mapping import JoystickAxes
from template_joystick_control.channel import Channel

from numpy.typing import ArrayLike


def joystick_body(
    joystick_message: sensor_msgs.msg.Joy,
        axes: JoystickAxes) -> Tuple[ArrayLike, Channel]:
    """
    sends `[surge, sway, yaw]` commands relative to body regardless of the orientation wrt north
    """
    # Tuple[np.ndarray, Channel]:
    yaw_command = joystick_message.axes[axes.RIGHT_X]
    surge_command = joystick_message.axes[axes.LEFT_Y]
    sway_command = joystick_message.axes[axes.LEFT_X]

    tau = [surge_command, sway_command, yaw_command]

    return np.array(tau, dtype=float).T, Channel.joystick
