from numpy.typing import ArrayLike
from template_joystick_control.kinematics import inv_rotate
from template_joystick_control.mapping import JoystickAxes
from template_joystick_control.topic import Topic
from typing import Tuple, Literal
import numpy as np
import sensor_msgs.msg


def joystick_body(
        joystick: sensor_msgs.msg.Joy,
        axes: JoystickAxes,
        heading: float = 0) -> Tuple[ArrayLike, Literal[Topic.joystick]]:
    """
    sends `[surge, sway, yaw]` commands relative to body
    """
    surge_command = 2 * joystick.axes[axes.LEFT_Y]
    sway_command = 2 * joystick.axes[axes.LEFT_X]
    yaw_command = joystick.axes[axes.RIGHT_X]

    tau = inv_rotate(
        heading) @ np.array([surge_command, sway_command, yaw_command], dtype=float).T
    return np.array(tau, dtype=float).T, Topic.joystick
