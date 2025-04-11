import numpy as np
from numpy.typing import NDArray

from template_observer.wrap import wrap

S = np.array(
    [[0, -1, 0], [1, 0, 0], [0, 0, 0]], dtype=np.float64)

M = np.array([[16, 0, 0],
              [0, 24, 0.53],
              [0, 0.53, 2.8]])

D = np.array([[0.66, 0, 0],
              [0, 1.3, 2.8],
              [0, 0, 1.9]])

M_inv = np.linalg.inv(M)


def rotation_matrix(heading: np.float64):
    return np.array([
        [np.cos(heading), -np.sin(heading), 0],
        [np.sin(heading),  np.cos(heading), 0],
        [0, 0, 1]
    ], dtype=np.float64)


def backstepping_controller_og(
    eta_hat,
    nu_hat,
    eta_d,
    eta_ds,
    v_s,
    s_dot,
    K1_gain,
    K2_gain,
) -> NDArray[np.float64]:

    yaw_rate = nu_hat[2]
    heading = eta_hat[2]

    R = rotation_matrix(heading)

    eta_error = eta_hat - eta_d
    # V1 gradient
    z1_k = R.T @ eta_error

    # Desired alpha1
    alpha1 = -K1_gain @ z1_k + R.T @ eta_ds * v_s

    # s_dot = v_s + omega_s
    # derivative of alpha1
    z1_dot = nu_hat - R.T @ eta_ds * s_dot - yaw_rate * S @ z1_k
    alpha1_dot = -K1_gain @ z1_dot - yaw_rate * S @ R.T @ eta_ds * v_s

    z2_k = nu_hat - alpha1
    # control law
    debug_locals = f"""
    z1_k: {z1_k}
    K2_gain: 
    {K2_gain}
    """
    assert True, debug_locals
    tau = M @ alpha1_dot + D @ nu_hat - K2_gain @ z2_k
    return tau, eta_error


def backstepping_controller(observer, reference, K1_gain, K2_gain, eta) -> NDArray[np.float64]:
    eta_hat = np.array(observer.eta, dtype=np.float64)
    # eta_hat = np.array(eta, dtype=np.float64)
    nu_hat = np.array(observer.nu, dtype=np.float64)
    eta_d = np.array(reference.eta_d, dtype=np.float64)
    eta_ds = np.array(reference.eta_ds, dtype=np.float64)
    v_s = np.array(reference.v_s, dtype=np.float64)
    omega_s = np.array(reference.w, dtype=np.float64)
    K1_gain = np.diag(K1_gain)
    K2_gain = np.diag(K2_gain)

    s_dot = v_s + omega_s
    return backstepping_controller_og(eta_hat, nu_hat, eta_d, eta_ds, v_s, s_dot, K1_gain, K2_gain)
