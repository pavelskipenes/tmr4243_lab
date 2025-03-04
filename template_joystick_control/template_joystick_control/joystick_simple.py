#!/usr/bin/env python3

import sensor_msgs.msg
import numpy as np

from typing import Tuple
from numpy.typing import ArrayLike

from template_joystick_control.joystick_mapping import JoystickAxes
from template_joystick_control.channel import Channel


def joystick_simple(
        joystick_message: sensor_msgs.msg.Joy,
        axes: JoystickAxes) -> Tuple[ArrayLike, Channel]:
    """
    simple joystick controller sends raw actuation values for thrusters
    Thrust allocation are bypassed.
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

    return np.array(actuation, dtype=float).T, Channel.actuation
