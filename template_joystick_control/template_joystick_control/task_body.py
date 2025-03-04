from numpy.typing import ArrayLike
from template_joystick_control.channel import Channel
from template_joystick_control.mapping import JoystickAxes
from typing import Tuple
import numpy as np
import sensor_msgs.msg


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
