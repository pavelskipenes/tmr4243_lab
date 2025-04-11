from numpy.typing import NDArray
from template_observer.wrap import wrap
from typing import Any
import numpy as np
import tmr4243_interfaces.msg


def rotation_matrix(heading):
    R = np.array([[np.cos(heading), -np.sin(heading), 0],
                  [np.sin(heading),  np.cos(heading), 0],
                  [0, 0, 1]])
    return R


def PD_FF_controller(
    observer: tmr4243_interfaces.msg.Observer,
    reference: tmr4243_interfaces.msg.Reference,
    P_gain: NDArray[np.float64],
    I_gain: Any,
    D_gain: NDArray[np.float64],
    eta
) -> np.ndarray:
    # try to keep the same interface between controllers
    _ = I_gain

    # eta_hat = np.array(observer.eta)
    eta_hat = np.array(eta)
    nu_hat = np.array(observer.nu)
    eta_d = np.array(reference.eta_d)

    heading = eta_hat[2]
    R = rotation_matrix(heading)

    eta_error = eta_hat - eta_d
    eta_error[2] = wrap(eta_error[2])
    error_position_body = R.T @ eta_error

    tau = -P_gain @ error_position_body - D_gain @ nu_hat

    return tau
