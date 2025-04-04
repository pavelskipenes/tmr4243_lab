import numpy as np

skew_symmetric_matrix = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])


def rotation_matrix(psi):
    return np.array([[np.cos(psi), -np.sin(psi), 0],
                     [np.sin(psi),  np.cos(psi), 0],
                     [0, 0, 1]])


def get_desired_pose2(delta_p):
    return np.array([delta_p[0], delta_p[1], 0])


def get_velocity_profile(velocity_reference, distance_between_waypoints):
    return velocity_reference / distance_between_waypoints


def get_distance_between_waypoints(point_start, point_end):
    delta_p = point_end - point_start
    return np.linalg.norm(delta_p)


def backstepping_controller(observer, reference, K1_gain, K2_gain) -> np.ndarray:

    # Getting the states from the observer
    eta_hat = observer.eta
    nu_hat = observer.nu
    bias_hat = observer.bias

    # Getting the states from the reference
    eta_d = reference.eta_d
    eta_ds = reference.eta_ds
    eta_ds2 = reference.eta_ds2

    w = reference.w
    v_s = reference.v_s  # velocity reference
    v_ss = reference.v_ss  # acceleration reference

    M = np.array([[16, 0, 0],
                  [0, 24, 0.53],
                  [0, 0.53, 2.8]])

    D = np.array([[0.66, 0, 0],
                  [0, 1.3, 2.8],
                  [0, 0, 1.9]])
    heading = eta_hat[2]
    R = rotation_matrix(heading)
    current_pose_error_body = eta_hat - eta_d

    point_start = eta_hat
    point_end = eta_d
    delta_p = point_end - point_start

    path_velocity = get_desired_pose2(delta_p)

    distance_between_waypoints = get_distance_between_waypoints(eta_hat, eta_d)
    velocity_profile = get_velocity_profile(
        v_s, distance_between_waypoints)

    alpha1 = -K1_gain @ current_pose_error_body + \
        R.T @ path_velocity * velocity_profile
    current_velocity_error_body = nu_hat - alpha1

    yaw_rate = nu_hat[2]

    path_velocity_based_on_distance = - \
        (eta_hat - eta_d).T @ path_velocity

    # Omega_s (use gradient law) update law
    mu = 1.0
    omega_s = -mu * path_velocity_based_on_distance

    # Path parameter update
    dot_s = velocity_profile + omega_s
    pose_error_body_dot = yaw_rate - R.T @ path_velocity * dot_s - \
        yaw_rate * skew_symmetric_matrix @ current_pose_error_body
    dot_alpha1 = -K1_gain @ pose_error_body_dot - yaw_rate * \
        skew_symmetric_matrix @ R.T @ path_velocity * velocity_profile
    tau = M @ dot_alpha1 + D @ nu_hat - \
        K2_gain @ current_velocity_error_body
    return tau

