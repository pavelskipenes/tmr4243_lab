from numpy import pi
from numpy.typing import ArrayLike
from template_joystick_control.mapping import JoystickAxes
from template_joystick_control.task_body import joystick_body
from template_joystick_control.topic import Topic
from typing import Tuple, Literal
import sensor_msgs.msg


def joystick_basin(
        joystick: sensor_msgs.msg.Joy,
        axes: JoystickAxes,
        heading: float) -> Tuple[ArrayLike, Literal[Topic.joystick]]:
    """
    return [`surge` `sway` `yaw`] commands relative to `heading`
    """

    offset = 0
    return joystick_body(joystick, axes, heading + offset)
