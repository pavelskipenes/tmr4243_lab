import numpy as np
from numpy.typing import NDArray
from typing import List
from template_observer.wrap import wrap


def rotation_matrix(psi: float) -> NDArray[np.float64]:
    return np.array([
        [np.cos(psi), -np.sin(psi), 0.0],
        [np.sin(psi), np.cos(psi), 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)


class Luenberger():
    damping: NDArray[np.float64]
    mass: NDArray[np.float64]
    mass_inv: NDArray[np.float64]
    bias_time_constants: NDArray[np.float64]
    Tb_inv: NDArray[np.float64]
    dt: float
    state_estimate_current: NDArray[np.float64]

    def __init__(self,
                 damping: NDArray[np.float64],
                 mass: NDArray[np.float64],
                 observer_gains: NDArray[np.float64],
                 time_step: float,
                 bias_time_constants: NDArray[np.float64],
                 initial_state_estimate: NDArray[np.float64] = np.array(
                     np.zeros(9), dtype=np.float64),
                 ) -> None:

        # todo: make this into an input

        assert np.shape(bias_time_constants) == (
            3, 1), np.shape(bias_time_constants)

        assert np.shape(damping) == (3, 3)
        self.damping = damping

        assert np.shape(mass) == (3, 3)
        self.mass_inv = np.linalg.inv(mass)

        self.dt = time_step

        assert np.shape(initial_state_estimate) == (
            9, 1), np.shape(initial_state_estimate)
        self.state_estimate_current = initial_state_estimate

        self.L1: NDArray[np.float64] = np.diag(
            [1/val * 2 * np.pi for val in bias_time_constants.flatten().tolist()])
        assert np.shape(self.L1) == (3, 3), np.shape(self.L1)

        self.bias_time_constants = np.diag(
            bias_time_constants.flatten().tolist())

        self.L2: NDArray[np.float64] = np.diag(np.diag(mass))

        assert self.L2.shape == (3, 3)

        assert observer_gains.shape == (1, 3)

        self.L3: NDArray[np.float64] = self.L2 * observer_gains.T
        assert self.L3.shape == (3, 3)

    def observe(
        self,
        measurement: NDArray[np.float64],
        actuation_input: NDArray[np.float64],
    ) -> List[float]:
        assert measurement.shape == (3, 1)

        heading: float = measurement.flatten()[2]
        assert isinstance(heading, float)

        R = rotation_matrix(wrap(heading))
        assert R.shape == (3, 3)

        pose_estimate_current = self.state_estimate_current[0:3]
        velocity_estimate_current = self.state_estimate_current[3:6]
        bias_estimate_current = self.state_estimate_current[6:9]

        assert pose_estimate_current.shape == (3, 1)
        assert velocity_estimate_current.shape == (3, 1)
        assert bias_estimate_current.shape == (3, 1)

        measurement_error = measurement - pose_estimate_current

        assert measurement_error.shape == (3, 1)

        pose_estimate_dot = R @ velocity_estimate_current + self.L1 @ measurement_error

        assert pose_estimate_dot.shape == (3, 1)

        velocity_estimate_dot = self.mass_inv @ (
            -self.damping @ velocity_estimate_current + bias_estimate_current +
            actuation_input + self.L2 @ R.T @ measurement_error
        )

        assert velocity_estimate_dot.shape == (3, 1)
        assert self.L3.shape == (3, 3)
        assert R.shape == (3, 3)
        assert measurement_error.shape == (3, 1)
        assert bias_estimate_current.shape == (3, 1)
        assert self.bias_time_constants.shape == (3, 3)

        bias_estimate_dot = -self.bias_time_constants @ bias_estimate_current + \
            self.L3 @ R.T @ measurement_error

        assert bias_estimate_dot.shape == (3, 1)

        # TODO: replace with RK45
        pose_estimate_next = pose_estimate_current + self.dt * pose_estimate_dot
        velocity_estimate_next = velocity_estimate_current + self.dt * velocity_estimate_dot
        bias_estimate_next = bias_estimate_current + self.dt * bias_estimate_dot

        assert pose_estimate_next.shape == (3, 1)
        assert velocity_estimate_next.shape == (3, 1)
        assert bias_estimate_next.shape == (3, 1)

        state_estimate_current = np.vstack((
            pose_estimate_next,
            velocity_estimate_next,
            bias_estimate_next,
        ), dtype=np.float64)

        assert state_estimate_current.shape == (9, 1)

        self.state_estimate_current = state_estimate_current
        # this pience of shit code just returns the column state estimate as a list of elements
        # there is probably a much nicer way of doing this
        return np.array(self.state_estimate_current.T.tolist()).flatten().tolist()
