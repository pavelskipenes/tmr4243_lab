from numpy.typing import NDArray
from template_thrust_allocation.thruster_positions import ThrusterPositions
import numpy as np
from numpy.linalg import inv


class ThrustAllocator():
    def __init__(self, thruster_positions: ThrusterPositions):
        l_0x, l_1x, l_1y, l_2x, l_2y = thruster_positions.get_thruster_positions()
        """
        Thrust configuration, 3x5 matrix.
        Describes how the vessels thrusters are configured on the vessel.

        Math notation:
            B

        See also:
           Lecture note on thrust allocation for rectangular thrust configuration matrix

        Tuning:
            No tuning should be done here.
        """
        self._extended_thrust_conf = np.array([
            [0, 1, 0, 1, 0],
            [1, 0, 1, 0, 1],
            [l_0x, -l_1y, l_1x, -l_2y, l_2x]
        ], dtype=np.float64)

        """
        Thruster gains, 5x1 matrix.

        Math notation:
            K

        Tuning:
            Increasing values will increase produced thrust
            Decreasing values will decrease produced thrust
        """
        self._gain = np.diag(
            [
                1.0,  # 2.629,    # Tunnel thruster
                1.0,  # 1.03,     # Azimuth 0 Force magnitude
                1.0,  # 1.03,     # Azimuth 1 Force Magnitude
                1.0,  # 1.03,     # Azimuth 0 Angle
                1.0,  # 1.03      # Azimuth 0 Angle
            ]
        )
        """
        Force desired, 5x1 matrix.
        
        Math notation:
            F_d

        Tuning:
            Define bias forces and angles in thrust optimization.
            These values will be used iteratevely as a basis for optimization problem.
        """
        self._force_desired: NDArray = np.array(
            [
                0.0,  # Tunnel
                0.0,  # Azimuth 0 magnitude
                0.0,  # Azimuth 1 Magnitude
                0.0,  # Azimuth 0 Angle
                0.0,  # Azimuth 1 Angle
            ], dtype=np.float64)

        """
        Weights.

        Math notation:
            W

        Tuning:
            Increasing values penalizes the component more
            descreasing values penalizes values less.
            All tuning values are expected to be greater than 1
        """
        self._weights = np.diag(
            [
                1.0,  # Tunnel
                1.0,  # Azimuth 0 x
                1.0,  # Azimuth 0 y
                1.0,  # Azimuth 1 x
                1.0,  # Azimuth 1 y
            ])

        self._force_bias_last = np.array([0, 0, 0, 0, 0]).T

    @property
    def gain(self) -> NDArray:
        """
        Get gain.
        Math notation: K

        Output:
            5x5 matrix.
        """
        return self._gain

    @property
    def gain_inv(self) -> NDArray:
        """
        Get inverted gain
        Math notation: K^{-1}.

        Output:
            5x5 matrix.
        """
        return inv(self.gain)

    @property
    def weights(self) -> NDArray:
        """
        Get weighting matrix.
        Math notation: W.

        Output:
            5x5 matrix.
        """
        return self._weights

    @property
    def weights_inv(self) -> NDArray:
        """
        Get inverted weighting matrix. W^{-1} in math notation
.
        Output:
            5x5 matrix.
        """
        return inv(self._weights)

    @property
    def extended_thrust_conf(self) -> NDArray:
        """
        Get extended thruster configuration matrix.
        Math notation: B.

        Output:
            3x5 matrix.
        """
        return self._extended_thrust_conf

    @property
    def weighted_pseudoinverse(self) -> NDArray:
        """
        Weighted Moore-Penrose pseudoinverse.
        Math notation: W.

        Output:
            5x3 matrix.

        See also:
            Lecure notes 5 on "DP control design" - page 15
        """

        # to avoid singulareties
        small_number = 1e-11

        return (
            self.weights_inv @
            self.extended_thrust_conf.T @
            inv(
                self.extended_thrust_conf @
                self.weights_inv @
                self.extended_thrust_conf.T +
                small_number * np.eye(3)
            )
        )

    def force_bias(self) -> NDArray:
        """
        Calculate the bias force.

        Side effect:
            stores calculated value for next iteration of this call

        Output:
            5x1 column vector
        """
        current_force_bias = (
            np.eye(5) -
            self.weighted_pseudoinverse @
            self.extended_thrust_conf
        ) @ self._force_bias_last
        self._force_bias_last = current_force_bias
        return current_force_bias

    def allocate_extended(self, tau_cmd: NDArray) -> NDArray:
        """
        get optimal allocated thrust using extended thrust allocation matrix

        Input:
            3x1 matrix
            [surge, sway, yaw] request

        Output:
            5x1 matrix
            [
                tunnel_magnitude in range [-1, 1],
                aximuth_0_magnitude in range [0, 1],
                azimuth_1_magnitude in range [0, 1],
                azimuth_0_angle in range [-pi, pi],
                azimuth_1_angle in range [-pi, pi],
            ]
        """
        force_optimal = self.weighted_pseudoinverse @ tau_cmd + self.force_bias()
        actuation_optimal = self.gain_inv @ force_optimal

        return np.array(
            [
                # Tunnel
                actuation_optimal[0],
                # Aximuth 0 magnitude
                np.sqrt(
                    actuation_optimal[1]**2 +
                    actuation_optimal[2]**2
                ),
                # Azimuth 1 magnitude
                np.sqrt(
                    actuation_optimal[3]**2 + actuation_optimal[4]**2
                ),
                # Azimuth angles
                np.arctan2(actuation_optimal[2], actuation_optimal[1]),
                np.arctan2(actuation_optimal[4], actuation_optimal[3]),
            ]
        )
