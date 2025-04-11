from numpy.typing import NDArray
import numpy as np


def update_law(
        eta_error: NDArray[np.float64],
        eta_ds: NDArray[np.float64],
        v_s: np.float64) -> tuple[np.float64, np.float64, np.float64]:

    V1s = np.float64(-eta_error.T @ eta_ds)
    mu = np.float64(1.0)
    omega_s = -mu * V1s

    # debug_locals = f"""
    # omega_s = {omega_s}
    # mu = {mu}
    # V1s = {V1s}
    # eta_error = {eta_error}
    # eta_ds = {eta_ds}
    # """
    # assert omega_s >= 0, debug_locals

    v_ss = np.float64(0.0)
    return omega_s, v_s, v_ss
