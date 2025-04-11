import numpy as np
from numpy.typing import NDArray
from template_guidance.update_law import update_law


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
    np.clip(s, 0, 1)
    return s, s_dot, v_s, v_ss
