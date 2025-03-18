from numpy.typing import ArrayLike
from template_joystick_control.mapping import JoystickAxes
from template_joystick_control.topic import Topic
from typing import Tuple, Literal
import numpy as np
import sensor_msgs.msg


def joystick_simple(
        joystick_message: sensor_msgs.msg.Joy,
        axes: JoystickAxes) -> Tuple[ArrayLike, Literal[Topic.actuation]]:
    """
    returns Actuation and Topic.actuation where the message is intended to be sent to

    this task controls all actuators manually bypassing thrust allocation
    """
    tunnel_thruster_actuation = (joystick_message.axes[axes.TRIGGER_RIGHT] -
                                 joystick_message.axes[axes.TRIGGER_LEFT]) / 2.0

    azimuth_starboard_actuation = np.linalg.norm(np.array((
        joystick_message.axes[axes.LEFT_X], joystick_message.axes[axes.LEFT_Y]
    )))
    azimuth_starboard_angle = np.arctan2(
        joystick_message.axes[axes.LEFT_X], joystick_message.axes[axes.LEFT_Y])

    azimuth_port_actuation = np.linalg.norm(np.array((
        joystick_message.axes[axes.RIGHT_X], joystick_message.axes[axes.RIGHT_Y]
    )))
    azimuth_port_angle = np.arctan2(
        joystick_message.axes[axes.RIGHT_X], joystick_message.axes[axes.RIGHT_Y])

    actuation = [
        tunnel_thruster_actuation,
        azimuth_starboard_actuation,
        azimuth_port_actuation,
        azimuth_starboard_angle,
        azimuth_port_angle,
    ]

    return np.array(actuation, dtype=float).T, Topic.actuation
