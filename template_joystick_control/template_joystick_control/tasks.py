import numpy as np
import sensor_msgs.msg

from enum import Enum
from channel import Channel
from joystick_emulated import RandomWalk, joystick_emulator
from typing import Optional
from std_msgs.msg import Float32MultiArray
from joystick_mapping import JoystickButtons, JoystickAxes
from joystick_simple import joystick_simple
from joystick_force_basin_relative import joystick_basin
from joystick_force_body_relative import joystick_body


class Task(str, Enum):
    SIMPLE = "simple",  # direct thruster actuation
    BODY = "body",  # relative to the vessel
    BASIN = "basin",  # relative to the virtual north
    EMULATED = "emulated"


# used by dynamic parameters in control node.
# TASKS = [Task.SIMPLE, Task.BODY, Task.BASIN]
# possible dynamic alternative:
# list(Task.__members__)

def get_new_requested_task(
        joystick_message: sensor_msgs.msg.Joy,
        buttons: JoystickButtons) -> Optional[Task]:
    """
    Returns a new Joystick task (joystick input interpreter) if that has been requested by the operator.
    Otherwise `None` is returned.
    """

    print(buttons.__slots__)
    print(type(buttons))
    assert buttons.TRIANGLE is not None
    assert isinstance(buttons, JoystickButtons)
    # Note: The buttons generally needs debouncing. Not needed here.
    if joystick_message.buttons[buttons.TRIANGLE]:
        return Task.SIMPLE
    if joystick_message.buttons[buttons.SQUARE]:
        return Task.BODY
    if joystick_message.buttons[buttons.CIRCLE]:
        return Task.BASIN
    return None


def get_actuation_and_channel(
        msg: sensor_msgs.msg.Joy,
        buttons: JoystickButtons,
        axes: JoystickAxes,
        random_walk: RandomWalk,
        last_eta_msg: Float32MultiArray,
        task_default: Task
) -> Optional[tuple[np.ndarray, Channel]]:
    """
    returns the actuation and what channel it should be published on.
    Can return `None` if the input arguments is not sufficient in producing an output.
    """
    match get_new_requested_task(msg, buttons) or task_default:
        case Task.SIMPLE:
            return joystick_simple(msg, axes)
        case Task.BODY:
            return joystick_body(msg, axes)
        case Task.BASIN:
            return joystick_basin(msg, axes, last_eta_msg)
        case Task.EMULATED:
            return joystick_emulator(random_walk)
        case _:
            raise Exception("unreachable")
