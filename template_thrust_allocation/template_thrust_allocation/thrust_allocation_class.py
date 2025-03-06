import numpy as np
from dataclasses import dataclass

@dataclass
class ThrusterPositions:
    l_0 = 0.370  # m
    l_1x = -0.415
    l_1y = -0.07
    l_2x = -0.415
    l_2y = 0.07
    def return_values(self):
        return self.l_0, self.l_1x, self.l_1y, self.l_2x, self.l_2y

class ThrustAllocator():
    def __init__(self, thruster_positions: ThrusterPositions):
        self.thruster_positions = thruster_positions
        self.epsilon = 1e-9 # Small value used in weighted pseudoinverse to avoid singularity, from lecture video on pseudoinvere in marine controlsystems 1
        self.K = np.diag([2.629, 1.03, 1.03, 1.03, 1.03])  # Thrust coeffisient matrix - k0, k1, k2
        self.B_extended = self.construct_extended_config_matrix()
        W = np.diag([1, 1, 1, 1, 1]) # placeholder 
        self.B_pseudo = self.weighted_pseudoinverse(W)
        self.Fd = np.array([0, 0, 0, 0, 0])
    
    def construct_extended_config_matrix(self):
        """Constructs Extended Thruster configuration matrix (B-matrix)"""
        l_0, l_1x, l_1y, l_2x, l_2y = self.thruster_positions.return_values()
        B_extended = np.array([
            [0, 1, 0, 1, 0],
            [1, 0, 1, 0, 1],
            [l_0, -l_1y, l_1x, -l_2y, l_2x]
        ])
        return B_extended

    def weighted_pseudoinverse(self, W) -> np.ndarray:
        """ Weighted Moore-Penrose pseudoinverse - Obtained from lecure notes 5 on "DP control design" - page 15"""
        W_inv = np.linalg.inv(W)  
        B_T = np.transpose(self.B_extended)

        return W_inv @ B_T @ np.linalg.inv(self.B_extended @ W_inv @ B_T + self.epsilon*np.eye(3)) 
        
    def allocate_extended(self, tau_cmd: np.ndarray) -> np.ndarray:
        """Optimal forces F*"""
        F_star = self.B_pseudo @ tau_cmd + (np.eye(5) - self.B_pseudo @ self.B_extended) @ self.Fd
        u_star = np.linalg.inv(self.K) @ F_star
        u0 = u_star[0]
        u1 = np.sqrt(u_star[1]**2+u_star[2]**2)
        u2 = np.sqrt(u_star[3]**2+u_star[4]**2)
        alpha1 = np.arctan2(u_star[2], u_star[1])
        alpha2 = np.arctan2(u_star[4], u_star[3])
        u_cmd = np.array([u0, u1, u2, alpha1, alpha2])

        return u_cmd