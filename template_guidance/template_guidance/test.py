from template_controller.backstepping_controller import backstepping_controller_og as backstepping_controller
from template_guidance.close_enough import close_enough
from template_guidance.path import path
from template_guidance.straight_line import straight_line
import datetime
import matplotlib.pyplot as plt
import numpy as np


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

    return eta_log


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
            [4, 0],
            [4, 4],
            [0, 4],

            [0, 0],

            [-4, 0],
            [-4, 4],
            [0, 4],

            [0, 0],
            [0, -4],
        ])

    eta = loop(waypoints, M, D)

    date = datetime.datetime.now()
    _, axes = plt.subplots(1, 1)
    axes.plot(waypoints[:, 1], waypoints[:, 0], 'ro-', label="Waypoints")
    axes.plot([point[1] for point in eta], [point[0]
              for point in eta], 'b--', label="Vessel Path")
    axes.grid(True)
    axes.legend()
    axes.set_title(f"{date}")
    axes.axis("equal")
    plt.savefig("square_test.svg")


if __name__ == "__main__":
    main()
