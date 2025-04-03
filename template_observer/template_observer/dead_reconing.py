from numpy.typing import NDArray
from template_observer.abstract_observer import Observer
from template_observer.luenberger import Luenberger
from template_observer.raft import Raft
from typing import List
import numpy as np


class DeadReconing(Observer):
    """
    DeadReconing will use Luenberger without gains to estimate the position
    """

    def __init__(
        self,
            raft: Raft,
            time_step: float,
            bias_time_constants: NDArray[np.float64],
            L_1: NDArray[np.float64],
            L_2: NDArray[np.float64],
            L_3: NDArray[np.float64],
            initial_state_estimate: NDArray[np.float64],
    ) -> None:

        _ = L_1
        _ = L_2
        _ = L_3

        self.luenberger = Luenberger(
            raft=raft,
            time_step=time_step,
            bias_time_constants=bias_time_constants,
            L_1=np.eye(3) * 0,
            L_2=np.eye(3) * 0,
            L_3=np.eye(3) * 0,
            initial_state_estimate=initial_state_estimate,
        )

    def observe(self, measurement: NDArray[np.float64], actuation_input: NDArray[np.float64]) -> List[float]:
        return self.luenberger.observe(measurement, actuation_input)

    def get_name(self) -> str:
        return "dead reconing"
