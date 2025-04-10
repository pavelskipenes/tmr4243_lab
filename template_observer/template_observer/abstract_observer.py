from abc import ABC, abstractmethod
from numpy.typing import NDArray
from template_observer.raft import Raft
from typing import List
import numpy as np


class Observer(ABC):
    def __init__(self,
                 raft: Raft,
                 time_step: float,
                 bias_time_constants: NDArray[np.float64],
                 L_1: NDArray[np.float64],
                 L_2: NDArray[np.float64],
                 L_3: NDArray[np.float64],
                 initial_state_estimate: NDArray[np.float64]
                 ) -> None:
        assert raft.mass.shape == (3, 3)
        assert raft.damping.shape == (3, 3)
        assert time_step > 0
        assert bias_time_constants.shape == (3, 1)
        assert L_1.shape == (3, 3)
        assert L_2.shape == (3, 3)
        assert L_3.shape == (3, 3)
        assert initial_state_estimate.shape == (9, 1)
        # check if this code at all runs
        assert False

    @abstractmethod
    def observe(self, measurement: NDArray[np.float64], actuation_input: NDArray[np.float64]) -> List[float]:
        pass

    @abstractmethod
    def get_name(self) -> str:
        pass
