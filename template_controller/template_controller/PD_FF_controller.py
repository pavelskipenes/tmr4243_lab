
import numpy as np
from numpy.typing import NDArray
import tmr4243_interfaces.msg
from typing import Any


def rotation_matrix(heading):
    R = np.array([[np.cos(heading), -np.sin(heading), 0],
                  [np.sin(heading),  np.cos(heading), 0],
                  [0, 0, 1]])
    return R


def get_gains(M, D):
    # TODO: refactor out
    M = np.array([[16, 0, 0],
                  [0, 24, 0.53],
                  [0, 0.53, 2.8]])

    D = np.array([[0.66, 0, 0],
                  [0, 1.3, 2.8],
                  [0, 0, 1.9]])
    zeta = np.eye(3)
    T_n = 10
    w_n = (2*np.pi/T_n) * np.eye(3)

    P_gain = (w_n**2) @ M
    I_gain = np.eye(3) * 0
    D_gain = 2 * zeta @ w_n @ M - D

    return P_gain, D_gain, I_gain


def PD_FF_controller(
    observer: tmr4243_interfaces.msg.Observer,
    reference: tmr4243_interfaces.msg.Reference,
    P_gain: NDArray[np.float64],
    I_gain: Any,
    D_gain: NDArray[np.float64],
) -> np.ndarray:
    # try to keep the same interface between controllers
    _ = I_gain

    eta_hat = np.array(observer.eta)
    nu_hat = np.array(observer.nu)
    eta_d = np.array(reference.eta_d)

    heading = eta_hat[2]
    R = rotation_matrix(heading)

    error_position_body = R.T @ (eta_hat - eta_d)

    tau = -P_gain @ error_position_body - D_gain @ nu_hat

    return tau
