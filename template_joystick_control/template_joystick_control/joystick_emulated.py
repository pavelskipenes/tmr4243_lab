import random

import numpy as np

from typing import Tuple

from template_joystick_control.channel import Channel


class RandomWalk:
    def __init__(self, x_tune=0.1, y_tune=0.1, z_tune=1.0):
        self.x = random.uniform(-1, 1)
        self.y = random.uniform(-1, 1)
        self.z = random.uniform(-float('inf'), float('inf'))
        self.x_tune = x_tune
        self.y_tune = y_tune
        self.z_tune = z_tune

    def step(self) -> list:
        self.x += random.uniform(-self.x_tune, self.x_tune)
        self.y += random.uniform(-self.y_tune, self.y_tune)
        self.z += random.uniform(-self.z_tune, self.z_tune)

        self.x = max(-1, min(self.x, 1))
        self.y = max(-1, min(self.y, 1))

        return [self.x, self.y, self.z]


def joystick_emulator(random_walk: RandomWalk) -> Tuple[np.ndarray, Channel]:
    return np.array(random_walk.step(), dtype=float).T, Channel.joystick
