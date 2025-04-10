from numpy._typing import NDArray
import numpy as np


def stationkeeping() -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    eta_d = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    eta_ds = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    eta_ds2 = np.array([0.0, 0.0, 0.0], dtype=np.float64)

    return eta_d, eta_ds, eta_ds2

