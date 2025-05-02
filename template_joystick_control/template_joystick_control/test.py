from numpy import pi
from numpy.typing import ArrayLike, NDArray
from template_joystick_control.kinematics import inv_rotate
import numpy as np
from template_thrust_allocation.thrust_allocation_class import ThrustAllocator
from template_thrust_allocation.thruster_positions import ThrusterPositions

# Test cases for problem 4.3


def joystick_basin(
        joystick: NDArray,
        heading: float) -> ArrayLike:
    """
    return [`surge` `sway` `yaw`] commands relative to `heading`
    """

    offset = -pi/2
    return joystick_body(joystick, heading + offset)


def joystick_body(
        joystick: NDArray,
        heading: float = 0) -> ArrayLike:
    """
    sends `[surge, sway, yaw]` commands relative to body
    """
    surge_command = 2 * joystick[0]
    sway_command = 2 * joystick[1]
    yaw_command = joystick[2]

    tau = inv_rotate(
        heading) @ np.array([surge_command, sway_command, yaw_command], dtype=float).T
    return np.array(tau, dtype=float).T


def main() -> None:
    commands: list[NDArray] = [np.array([1, 0, 0]), np.array([0, 1, 0])]
    headings: list[float] = [-np.pi/2, np.pi]

    thrust_allocator: ThrustAllocator = ThrustAllocator(ThrusterPositions())

    for command, heading in zip(commands, headings):
        body = joystick_body(command)
        basin = joystick_basin(command, heading)
        body = np.round(body, decimals=2)
        basin = np.round(basin, decimals=2)
        # print("body:")
        # print(body)
        # print("basin:")
        # print(basin)
        u_body = thrust_allocator.allocate_extended(body)
        u_basin = thrust_allocator.allocate_extended(basin)
        # u_body = np.round(u_body, decimals=4)
        # u_basin = np.round(u_basin, decimals=4)
        print("u body:")
        print(u_body)
        print("u basin:")
        print(u_basin)


if __name__ == "__main__":
    main()
