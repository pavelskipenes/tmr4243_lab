#!/usr/bin/env python3

import numpy as np
from numpy._typing import NDArray
from template_observer.wrap import wrap


def rotation_matrix(psi):
    return np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi), np.cos(psi), 0],
        [0, 0, 1]
    ])


class Luenberger():
    L1: NDArray
    L2: NDArray
    L3: NDArray
    D: NDArray
    M: NDArray
    M_inv: NDArray
    Tb: NDArray
    Tb_inv: NDArray
    dt: float
    state_estimate_current: NDArray

    def __init__(self,
                 luenberger_gain: NDArray,
                 damping: NDArray,
                 mass: NDArray,
                 bias_time_constants: NDArray,
                 time_step: float,
                 initial_state_estimate: NDArray = np.array(np.zeros(9)),
                 ) -> None:

        # TODO: clean this shit
        self.L1 = luenberger_gain[0:2]
        self.L2 = luenberger_gain[3:5]
        self.L3 = luenberger_gain[6:8]
        self.D = damping
        self.M = mass
        self.M_inv = np.linalg.inv(self.M)
        self.Tb = bias_time_constants
        self.dt = time_step
        self.initial_state_estimate = initial_state_estimate

    def observe(
        self,
        measurement: NDArray,
        actuation_input: NDArray,
    ) -> NDArray:
        heading = measurement[2]
        R = rotation_matrix(wrap(heading))

        pose_estimate_current = self.state_estimate_current[0:3]
        measurement_error = measurement - pose_estimate_current

        velocity_estimate_current = self.state_estimate_current[3:6]
        pose_estimate_dot = R @ velocity_estimate_current + self.L1 @ measurement_error

        bias_estimate_current = self.state_estimate_current[6:9]
        velocity_estimate_dot = self.M_inv @ (
            -self.D @ velocity_estimate_current + bias_estimate_current +
            actuation_input + self.L2 @ R.T @ measurement_error
        )

        bias_estimate_dot = -self.Tb_inv @ bias_estimate_current + \
            self.L3 @ R.T @ measurement_error

        # TODO: replace with RK45
        pose_estimate_next = pose_estimate_current + self.dt * pose_estimate_dot
        velocity_estimate_next = velocity_estimate_current + self.dt * velocity_estimate_dot
        bias_estimate_next = bias_estimate_current + self.dt * bias_estimate_dot

        self.state_estimate_current = np.array([
            pose_estimate_next,
            velocity_estimate_next,
            bias_estimate_next
        ], dtype=np.float64).T

        return self.state_estimate_current
