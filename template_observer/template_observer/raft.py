import numpy as np


class Raft():
    mass = np.matrix(
        [
            [16, 0, 0],
            [0, 24, 0.53],
            [0, 0.53, 2.8],
        ], dtype=np.float64)
    damping = np.matrix(
        [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
        ], dtype=np.float64)
