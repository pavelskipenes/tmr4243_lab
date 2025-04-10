from dataclasses import dataclass
# docs: https://docs.ros.org/en/ros2_packages/jazzy/api/joy/index.html#ros-2-driver-for-generic-joysticks-and-game-controllers
# docs is not correct for playstaion 4 controller and the indexies had to be figured out manually.


# TOOD: check if frozen=True can be used
@dataclass(slots=True, init=True)
class JoystickButtons:
    CROSS: int = 0
    CIRCLE: int = 1
    TRIANGLE: int = 2
    SQUARE: int = 3
    SHOULDER_LEFT: int = 4
    SHOULDER_RIGHT: int = 5
    SHARE: int = 8
    OPTIONS: int = 9
    GUIDE: int = 10
    STICK_LEFT: int = 11
    STICK_RIGHT: int = 12

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
