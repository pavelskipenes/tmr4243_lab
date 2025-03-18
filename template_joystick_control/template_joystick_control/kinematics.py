import numpy as np


def inv_rotate(heading) -> np.matrix:
    def c(angle) -> np.float64:
        return np.cos(angle)

    def s(angle) -> np.float64:
        return np.sin(angle)

    return np.matrix(
        [
            [c(heading), s(heading), 0.0],
            [-s(heading), c(heading), 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64
    )
