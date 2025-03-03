#!/usr/bin/env python3

import sensor_msgs.msg
import numpy as np

from typing import Tuple, Optional
from channel import Channel
from joystick_mapping import JoystickAxes
from std_msgs.msg import Float32MultiArray


def rotate(heading) -> np.matrix:
    def c(angle) -> np.float64:
        return np.cos(angle)

    def s(angle) -> np.float64:
        return np.sin(angle)
    return np.matrix(
        [
            [c(heading), -s(heading), 0],
            [s(heading), c(heading), 0],
            [0, 0, 1],
        ]
    )


def joystick_basin(
        joystick: sensor_msgs.msg.Joy,
        axes: JoystickAxes,
        last_eta_msg: Optional[Float32MultiArray]) -> Optional[Tuple[np.ndarray, Channel]]:
    """
    thruster commands are rotates with respect to the pool / basin regardless of the orientation of the vessel.
    """
    if last_eta_msg is None:
        return None

    if len(last_eta_msg.data) != 3:
        return None

    position = last_eta_msg.data
    heading = position[2]

    yaw_command = joystick.axes[axes.RIGHT_X]
    surge_command = joystick.axes[axes.LEFT_Y]
    sway_command = joystick.axes[axes.LEFT_X]

    tau = rotate(heading) @ [surge_command, sway_command, yaw_command]

    return np.array(tau, dtype=float).T, Channel.joystick
