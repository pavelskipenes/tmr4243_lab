from dataclasses import dataclass


@dataclass(slots=True, init=False)
class JoystickMapping:
    LEFT_STICK_HORIZONTAL: int
    LEFT_STICK_VERTICAL: int
    RESERVED: int
    RIGHT_STICK_HORIZONTAL: int
    RIGHT_STICK_VERTICAL: int
    RESERVED2: int
    RESERVED3: int
    RESERVED4: int
    A_BUTTON: int
    X_BUTTON: int
    Y_BUTTON: int
    B_BUTTON: int
    LEFT_TRIGGER: int
    RIGHT_TRIGGER: int
