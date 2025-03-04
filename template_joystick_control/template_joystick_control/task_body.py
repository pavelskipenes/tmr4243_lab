from numpy.typing import ArrayLike
from template_joystick_control.mapping import JoystickAxes
from template_joystick_control.topic import Topic
from typing import Tuple
import numpy as np
import sensor_msgs.msg


def joystick_body(
    joystick_message: sensor_msgs.msg.Joy,
        axes: JoystickAxes) -> Tuple[ArrayLike, Topic]:
    """
    sends `[surge, sway, yaw]` commands relative to body regardless of the orientation wrt north
    """
    yaw_command = joystick_message.axes[axes.RIGHT_X]
    surge_command = joystick_message.axes[axes.LEFT_Y]
    sway_command = joystick_message.axes[axes.LEFT_X]

    tau = [surge_command, sway_command, yaw_command]

    return np.array(tau, dtype=float).T, Topic.joystick
