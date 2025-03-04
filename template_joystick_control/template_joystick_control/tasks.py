import sensor_msgs.msg

from typing import Optional, Tuple
from enum import Enum

from std_msgs.msg import Float32MultiArray

from template_joystick_control.channel import Channel
from template_joystick_control.task_emulated import RandomWalk, joystick_emulator
from template_joystick_control.mapping import JoystickButtons, JoystickAxes
from template_joystick_control.task_simple import joystick_simple
from template_joystick_control.task_basin import joystick_basin
from template_joystick_control.task_body import joystick_body
from numpy.typing import ArrayLike


class Task(str, Enum):
    SIMPLE = "simple",  # direct thruster actuation
    BODY = "body",  # relative to the vessel
    BASIN = "basin",  # relative to the virtual north
    EMULATED = "emulated"


def get_new_requested_task(
        joystick_message: sensor_msgs.msg.Joy,
        buttons: JoystickButtons) -> Optional[Task]:
    """
    Returns a new Joystick task (joystick input interpreter) if that has been requested by the operator.
    Otherwise `None` is returned.
    """

    # Note: The buttons generally needs debouncing. Not needed here.
    if joystick_message.buttons[buttons.TRIANGLE]:
        return Task.SIMPLE
    if joystick_message.buttons[buttons.SQUARE]:
        return Task.BODY
    if joystick_message.buttons[buttons.CIRCLE]:
        return Task.BASIN
    return None


def get_actuation_channel_new_task(
        msg: sensor_msgs.msg.Joy,
        buttons: JoystickButtons,
        axes: JoystickAxes,
        random_walk: RandomWalk,
        last_eta_msg: Float32MultiArray,
        task_default: Task
) -> Tuple[Optional[Tuple[ArrayLike, Channel]], Task]:
    """
    returns the actuation and what channel it should be published on.
    Can return `None` if the input arguments is not sufficient in producing an output.
    """
    task = get_new_requested_task(msg, buttons) or task_default
    match task:
        case Task.SIMPLE:
            return joystick_simple(msg, axes), task
        case Task.BODY:
            return joystick_body(msg, axes), task
        case Task.BASIN:
            return joystick_basin(msg, axes, last_eta_msg), task
        case Task.EMULATED:
            return joystick_emulator(random_walk), task
        case _:
            raise Exception("unreachable")
