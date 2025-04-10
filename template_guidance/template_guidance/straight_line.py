from numpy.typing import NDArray
import numpy as np


def straight_line(
        waypoints: NDArray[np.float64],
        s: np.float64) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], np.float64]:
    start: NDArray[np.float64] = waypoints[0]
    end: NDArray[np.float64] = waypoints[1]
    delta_p = end - start

    heading_reference = np.arctan2(delta_p[1], delta_p[0])
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
