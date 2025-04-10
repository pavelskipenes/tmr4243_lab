
import numpy as np
from numpy._typing import NDArray
import tmr4243_interfaces.msg


def PD_FF_controller(
        observer: tmr4243_interfaces.msg.Observer,
        reference: tmr4243_interfaces.msg.Reference,
        P_gain: np.float64,
        D_gain: np.float64) -> NDArray[np.float64]:

    tau = np.zeros((3, 1), dtype=np.float64)

    # Getting the states from the observer
    eta_hat = observer.eta
    nu_hat = observer.nu
    bias_hat = observer.bias

    # Getting the states from the refernce
    eta_d = reference.eta_d
    eta_ds = reference.eta_ds
    eta_ds2 = reference.eta_ds2
    w = reference.w
    v_s = reference.v_s
    v_ss = reference.v_ss

    return tau

