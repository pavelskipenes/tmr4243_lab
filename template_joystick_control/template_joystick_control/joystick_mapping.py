from dataclasses import dataclass
# docs: https://docs.ros.org/en/ros2_packages/jazzy/api/joy/index.html#ros-2-driver-for-generic-joysticks-and-game-controllers
# docs is not correct for playstaion 4 controller and the indexies had to be figured out manually.


@dataclass(slots=True, init=False)
class JoystickButtons:
    CROSS: int
    CIRCLE: int
    TRIANGLE: int
    SQUARE: int
    SHOULDER_LEFT: int
    SHOULDER_RIGHT: int
    SHARE: int
    OPTIONS: int
    GUIDE: int
    STICK_LEFT: int
    STICK_RIGHT: int

    @classmethod
    def get_names(cls):
        # __annotations__ does not work without __dict__ implementation
        return [
            "CROSS",
            "CIRCLE",
            "TRIANGLE",
            "SQUARE",
            "SHOULDER_LEFT",
            "SHOULDER_RIGHT",
            "SHARE",
            "OPTIONS",
            "GUIDE",
            "STICK_LEFT",
            "STICK_RIGHT",
        ]


@dataclass(slots=True, init=False)
class JoystickAxes:
    LEFT_X: int
    LEFT_Y: int
    TRIGGER_LEFT: int
    RIGHT_X: int
    RIGHT_Y: int
    TRIGGER_RIGHT: int
    DPAD_HORIZONTAL: int
    DPAD_VERTICAL: int

    @classmethod
    def get_names(cls):
        # __annotations__ does not work without __dict__ implementation
        return [
            "LEFT_X",
            "LEFT_Y",
            "TRIGGER_LEFT",
            "RIGHT_X",
            "RIGHT_Y",
            "TRIGGER_RIGHT",
            "DPAD_HORIZONTAL",
            "DPAD_VERTICAL",
        ]
