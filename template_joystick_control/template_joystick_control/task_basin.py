from numpy.typing import ArrayLike
from template_joystick_control.mapping import JoystickAxes
from template_joystick_control.topic import Topic
from template_joystick_control.kinematics import inv_rotate
from typing import Tuple, Literal
import numpy as np
import sensor_msgs.msg


def joystick_basin(
        joystick: sensor_msgs.msg.Joy,
        axes: JoystickAxes,
        heading: float) -> Tuple[ArrayLike, Literal[Topic.joystick]]:
    """
    comamnded directions that are interpreted with respect to basin / pool.
    If heading is 0 the effect will be the same as if user called joystick_body.
    """
    yaw_command = joystick.axes[axes.RIGHT_X]
    surge_command = joystick.axes[axes.LEFT_Y]
    sway_command = joystick.axes[axes.LEFT_X]

    tau = inv_rotate(
        heading) @ np.array([surge_command, sway_command, yaw_command], dtype=float).T

    return np.array(tau, dtype=float).T, Topic.joystick
