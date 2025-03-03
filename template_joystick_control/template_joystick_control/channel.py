from enum import Enum


class Channel(str, Enum):
    """
    Joystick module can publish in three modes.

    Bypass mode:
    Joystick sets the thrust and angles on each thruster individually and bypasses the thrust allocator.
    These commands are 5x1 and are equivalent to whatver thrust allocator would produce.

    Vessel relative and Basin relative:
    Joystick will publish users request and those command will be processed by thrust allocator. Dimension of the messages aren 3x1.
    """
    actuation = 'u_cmd'
    joystick = 'tau_cmd'
