# from template_controller.backstepping_controller import backstepping_controller_og as backstepping_controller
# from template_guidance.close_enough import close_enough
# from template_guidance.path import path
# from template_guidance.straight_line import straight_line
import datetime
import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray


# local copies
M = np.array([[16, 0, 0],
              [0, 24, 0.53],
              [0, 0.53, 2.8]])

D = np.array([[0.66, 0, 0],
              [0, 1.3, 2.8],
              [0, 0, 1.9]])

M_inv = np.linalg.inv(M)
S = np.array(
    [[0, -1, 0], [1, 0, 0], [0, 0, 0]], dtype=np.float64)


def close_enough(s) -> bool:
    return s >= 0.95


def backstepping_controller(
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
    # eta_error[2] = wrap(eta_error[2])
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
    return tau


def straight_line(
        waypoints: NDArray[np.float64],
        s: np.float64) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], np.float64]:
    start: NDArray[np.float64] = waypoints[0]
    end: NDArray[np.float64] = waypoints[1]
    delta_p = end - start

    # heading_reference = np.arctan2(delta_p[1], delta_p[0])
    heading_reference = 0

    eta_ds = np.array(
        [
            delta_p[0],
            delta_p[1],
            np.float64(0.0)
        ]
    )
    norm_p: np.float64 = np.float64(np.linalg.norm(delta_p))
    eta_d = np.array(
        [
            start[0] + delta_p[0] * s,
            start[1] + delta_p[1] * s,
            heading_reference
        ]
    )
    eta_ds2 = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    return eta_d, eta_ds, eta_ds2, norm_p


def path(
    eta_ds: NDArray[np.float64],
    s_old: np.float64,
    eta_error: NDArray[np.float64],
    U_ref: np.float64,
    norm_p: np.float64,
    delta_time: np.float64,
) -> tuple[np.float64, np.float64, np.float64, np.float64]:

    v_s = U_ref / norm_p
    omega_s, v_s, v_ss = update_law(eta_error, eta_ds, v_s)

    s_dot = v_s + omega_s
    # debug_locals = f"""
    # s_dot = {s_dot}
    # v_s = {v_s}
    # omega_s = {omega_s}
    # """
    # assert s_dot >= 0, debug_locals
    s = s_old + s_dot * delta_time
    return s, s_dot, v_s, v_ss


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


def rotation_matrix(heading: np.float64):
    return np.array([
        [np.cos(heading), -np.sin(heading), 0],
        [np.sin(heading),  np.cos(heading), 0],
        [0, 0, 1]
    ], dtype=np.float64)


def simulate_dynamics(eta_hat, nu_hat, tau, dt, M_inv, D):
    heading = eta_hat[2]
    R = rotation_matrix(heading)
    nu_dot = M_inv @ (-D @ nu_hat + tau)
    nu_next = nu_hat + nu_dot * dt
    eta_next = eta_hat + R @ nu_next * dt
    return nu_next, eta_next


def loop(waypoints, M, D):
    K1_gain = np.diag([6.8, 8.95, 1])
    K2_gain = np.diag([9.5, 9.5, 7])*2
    dt = np.float64(0.01)
    s = np.float64(0.0)
    U_ref = np.float64(0.5)
    # initial conditions
    eta_hat = np.array([0, 0, 0])
    nu_hat = np.array([0, 0, 0])
    M_inv = np.linalg.inv(M)

    eta_log = []
    s_log = []
    while len(waypoints) != 1:
        eta_d, eta_ds, eta_ds2, norm_p = straight_line(
            waypoints, s)
        _ = eta_ds2

        eta_error = eta_hat - eta_d

        if not close_enough(s):
            u_ref = U_ref
        else:
            u_ref = np.float64(0.0)
            waypoints = waypoints[1:]
            s = np.float64(0.0)

        s, s_dot, v_s, v_ss = path(
            eta_ds,
            s,
            eta_error,
            u_ref,
            norm_p,
            dt,
        )

        _ = v_ss

        tau = backstepping_controller(
            eta_hat,
            nu_hat,
            eta_d,
            eta_ds,
            v_s,
            s_dot,
            K1_gain,
            K2_gain,
            # D,
            # M
        )

        nu_hat, eta_hat = simulate_dynamics(eta_hat, nu_hat, tau, dt, M_inv, D)
        eta_log.append(eta_hat.tolist())
        s_log.append(s)

    return eta_log, s_log


def main():
    M = np.array([[16, 0, 0],
                  [0, 24, 0.53],
                  [0, 0.53, 2.8]])
    D = np.array([[0.66, 0, 0],
                  [0, 1.3, 2.8],
                  [0, 0, 1.9]])

    waypoints = np.array(
        [

            [0, 0],
            [4, 4],
            # [0, 4],
            #
            # [0, 0],
            #
            # [-4, 0],
            # [-4, 4],
            # [0, 4],
            #
            # # [0, 0],
            #
            # [0, -4],
        ])

    eta, s_log = loop(waypoints, M, D)

    date = datetime.datetime.now()
    _, axes = plt.subplots(2, 1)
    axes[0].plot(waypoints[:, 1], waypoints[:, 0], 'ro-', label="Waypoints")
    axes[0].plot([point[1] for point in eta], [point[0]
                                               for point in eta], 'b--', label="Vessel Path")
    axes[0].grid(True)
    axes[0].legend()
    axes[0].set_title(f"{date}")
    axes[0].axis("equal")

    axes[1].plot(s_log, label="s")
    axes[1].grid(True)
    axes[1].legend()
    plt.savefig("square_test.svg")


if __name__ == "__main__":
    main()
