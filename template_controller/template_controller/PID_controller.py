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


sum_errors = np.array([0, 0, 0], dtype=np.float64)


def _PID_controller(eta_hat, nu_hat, eta_d, P_gain, I_gain, D_gain):
    heading = eta_hat[2]
    R = rotation_matrix(heading)

    dt = 0.01

    current_error = R.T @ (eta_hat - eta_d)
    global sum_errors
    sum_errors += current_error*dt

    tau = -I_gain @ sum_errors - P_gain @ current_error - D_gain @ nu_hat
    return tau


def PID_controller(
        observer: tmr4243_interfaces.msg.Observer,
        reference: tmr4243_interfaces.msg.Reference,
        P_gain: float,
        I_gain: float,
        D_gain: float) -> NDArray[np.float64]:
    eta_hat = np.array(observer.eta)
    nu_hat = np.array(observer.nu)
    eta_d = np.array(reference.eta_d)

    return _PID_controller(eta_hat, nu_hat, eta_d, P_gain, I_gain, D_gain)


def test_pid():
    P_gain, I_gain, D_gain = get_gains(0, 0)

    test_inputs = [
        # eta, nu, eta_d
        ([0, 0, 0], [0, 0, 0], [1, 0, 0]),
        ([0, 0, 0], [0, 0, 0], [0, 1, 0]),
        ([0, 0, 0], [0, 0, 0], [0, 0, 1]),

    ]
    for eta_hat, nu_hat, eta_d in test_inputs:
        eta_hat = np.array(eta_hat)
        nu_hat = np.array(nu_hat)
        eta_d = np.array(eta_d)

        tau = _PID_controller(
            eta_hat, nu_hat, eta_d, P_gain, I_gain, D_gain)
        message = f"eta: {eta_hat}, nu: {nu_hat}, eta_d: {eta_d}, tau: {tau}"
        print(message)


if __name__ == "__main__":
    test_pid()
