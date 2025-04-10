import numpy as np
from numpy._typing import NDArray
import tmr4243_interfaces.msg


def rotation_matrix(heading):
    R = np.array([[np.cos(heading), -np.sin(heading), 0],
                  [np.sin(heading),  np.cos(heading), 0],
                  [0, 0, 1]])
    return R


def get_gains(M, D) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    # TODO: move out to a shared module
    M = np.array([[16, 0, 0],
                  [0, 24, 0.53],
                  [0, 0.53, 2.8]])

    D = np.array([[0.66, 0, 0],
                  [0, 1.3, 2.8],
                  [0, 0, 1.9]])

    zeta = np.eye(3)
    T_n = 10
    w_n = (2 * np.pi/T_n) * np.eye(3)

    P_gain = (w_n**2) @ M
    D_gain = 2 * zeta @ w_n @ M - D
    I_gain = 0.1 * P_gain @ w_n
    return P_gain, I_gain, D_gain


def PID_controller(
        observer: tmr4243_interfaces.msg.Observer,
        reference: tmr4243_interfaces.msg.Reference,
        P_gain: float,
        I_gain: float,
        D_gain: float) -> NDArray[np.float64]:
    eta_hat = np.float64(observer.eta)
    nu_hat = np.float64(observer.nu)
    eta_d = np.float64(reference.eta_d)

    heading = eta_hat[2]
    R = rotation_matrix(heading)

    dt = 0.1

    error_position_body = R.T @ (eta_hat - eta_d)
    xi = error_position_body*dt

    tau = -I_gain @ xi - P_gain @ error_position_body - D_gain @ nu_hat
    return tau
