import numpy as np


def wrap(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi

