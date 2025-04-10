from numpy.typing import NDArray
import numpy as np


def update_law(
        eta_error: NDArray[np.float64],
        eta_ds: NDArray[np.float64],
        v_s: np.float64) -> tuple[np.float64, np.float64, np.float64]:

    random_variable: np.float64 = np.float64(-eta_error.T @ eta_ds)
    mu = np.float64(1.0)
    omega_s = -mu * random_variable

    v_ss = np.float64(0.0)
    return omega_s, v_s, v_ss
