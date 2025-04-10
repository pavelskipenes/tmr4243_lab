from numpy.typing import NDArray
from template_observer.abstract_observer import Observer
from template_observer.raft import Raft
from template_observer.wrap import wrap
from typing import List
import numpy as np


def rotation_matrix(psi: float) -> NDArray[np.float64]:
    return np.array([
        [np.cos(psi), -np.sin(psi), 0.0],
        [np.sin(psi), np.cos(psi), 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)


class Luenberger(Observer):
    damping: NDArray[np.float64]
    mass: NDArray[np.float64]
    mass_inv: NDArray[np.float64]
    bias_time_constants: NDArray[np.float64]
    Tb_inv: NDArray[np.float64]
    dt: float
    state_estimate_current: NDArray[np.float64]

    def __init__(self,
                 raft: Raft,
                 time_step: float,
                 bias_time_constants: NDArray[np.float64],
                 L_1: NDArray[np.float64],
                 L_2: NDArray[np.float64],
                 L_3: NDArray[np.float64],
                 initial_state_estimate: NDArray[np.float64] = np.array(
                     np.zeros(9), dtype=np.float64),
                 ) -> None:
        np.seterr(all='raise')
        assert np.shape(bias_time_constants) == (
            3, 1), np.shape(bias_time_constants)

        assert np.shape(raft.damping) == (3, 3)
        self.damping = raft.damping

        assert np.shape(raft.mass) == (3, 3)
        self.mass_inv = np.linalg.inv(raft.mass)

        self.dt = time_step

        assert np.shape(initial_state_estimate) == (
            9, 1), np.shape(initial_state_estimate)
        self.state_estimate_current = initial_state_estimate

        self.bias_time_constants = np.diag(
            bias_time_constants.flatten().tolist())

        assert L_1.shape == (3, 3)
        assert L_2.shape == (3, 3)
        assert L_3.shape == (3, 3)

        self.L1 = L_1
        self.L2 = L_2
        self.L3 = L_3

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

    def get_name(self) -> str:
        return "luenberger"
