from enum import Enum
from numpy.typing import ArrayLike
from template_joystick_control.mapping import JoystickButtons, JoystickAxes
from template_joystick_control.task_basin import joystick_basin
from template_joystick_control.task_body import joystick_body
from template_joystick_control.task_simple import joystick_simple
from template_joystick_control.topic import Topic
from typing import Optional, Tuple
import sensor_msgs.msg


class Task(str, Enum):
    SIMPLE = "simple",  # direct thruster actuation
    BODY = "body",  # relative to the vessel
    BASIN = "basin",  # relative to the virtual north


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


def get_actuation_topic_new_task(
        msg: sensor_msgs.msg.Joy,
        buttons: JoystickButtons,
        axes: JoystickAxes,
        heading: float,
        task_default: Task
) -> Tuple[Tuple[ArrayLike, Topic], Task]:
    """
    returns the command (or actuation for the simple task) and the intended topic it should be published on.
    This function will never return any error. but trash is = trash out.
    """
    task = get_new_requested_task(msg, buttons) or task_default
    match task:
        case Task.SIMPLE:
            return joystick_simple(msg, axes), task
        case Task.BODY:
            return joystick_body(msg, axes, 0), task
        case Task.BASIN:
            return joystick_basin(msg, axes, heading), task
        case _:
            raise Exception("unreachable")
