from dataclasses import dataclass
# docs: https://docs.ros.org/en/ros2_packages/jazzy/api/joy/index.html#ros-2-driver-for-generic-joysticks-and-game-controllers
# deviations from docs are noted as comments

@dataclass(slots=True, init=False)
class JoystickButtons:
    A: int
    B: int
    X: int
    Y: int
    BACK: int
    GUIDE: int
    START: int
    STICK_LEFT: int
    STICK_RIGHT: int
    SHOULDER_LEFT: int
    SHOULDER_RIGHT: int
    DPAD_UP: int
    DPAD_DOWN: int
    DPAD_LEFT: int
    DPAD_RIGHT: int
    MISC1: int
    PADDLE1: int
    PADDLE2: int
    PADDLE3: int
    PADDLE4: int
    TOUCHPAD: int


@dataclass(slots=True, init=False)
class JoystickAxes:
    LEFT_X: int
    LEFT_Y: int
    TRIGGER_LEFT: int # index 4 in docs. index 2 in reality 🤷
    RIGHT_X: int
    RIGHT_Y: int
    TRIGGER_RIGHT: int
