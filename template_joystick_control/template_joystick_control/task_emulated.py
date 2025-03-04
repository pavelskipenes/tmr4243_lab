import random

import numpy as np

from typing import Tuple

from template_joystick_control.channel import Channel
from numpy.typing import ArrayLike


class RandomWalk:
    def __init__(self, x_tune=0.2, y_tune=0.2, z_tune=0.1):
        self.surge = random.uniform(-1, 1)
        self.sway = random.uniform(-1, 1)
        self.yaw = random.uniform(-1, 1)
        self.surge_tune = x_tune
        self.sway_tune = y_tune
        self.yaw_tune = z_tune

    def step(self) -> list:
        self.surge += random.uniform(-self.surge_tune, self.surge_tune)
        self.sway += random.uniform(-self.sway_tune, self.sway_tune)
        self.yaw += random.uniform(-self.yaw_tune, self.yaw_tune)

        self.surge = max(-1, min(self.surge, 1))
        self.sway = max(-1, min(self.sway, 1))
        self.yaw = max(-1, min(self.yaw, 1))

        return [self.surge, self.sway, self.yaw]


def joystick_emulator(random_walk: RandomWalk) -> Tuple[ArrayLike, Channel]:
    """
    Slowly varying input for those plebs without a joystick
    Sends `[surge, sway, yaw].T` commands
    """
    return np.array(random_walk.step(), dtype=float).T, Channel.joystick
