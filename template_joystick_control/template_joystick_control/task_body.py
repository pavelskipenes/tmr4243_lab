from numpy.typing import ArrayLike
from template_joystick_control.mapping import JoystickAxes
from template_joystick_control.tasks import joystick_basin
from template_joystick_control.topic import Topic
from typing import Tuple, Literal
import sensor_msgs.msg


def joystick_body(
    joystick_message: sensor_msgs.msg.Joy,
        axes: JoystickAxes) -> Tuple[ArrayLike, Literal[Topic.joystick]]:
    """
    sends `[surge, sway, yaw]` commands relative to body regardless of the orientation wrt north
    """
    return joystick_basin(joystick_message, axes, 0)
