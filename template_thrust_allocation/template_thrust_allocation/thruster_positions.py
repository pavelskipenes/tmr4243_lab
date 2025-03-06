from dataclasses import dataclass
from typing import Tuple


@dataclass
class ThrusterPositions:
    l_0x: float = 0.370
    l_1x: float = -0.415
    l_1y: float = -0.07
    l_2x: float = -0.415
    l_2y: float = 0.07

    def get_thruster_positions(self) -> Tuple[float, float, float, float, float]:
        return self.l_0x, self.l_1x, self.l_1y, self.l_2x, self.l_2y
