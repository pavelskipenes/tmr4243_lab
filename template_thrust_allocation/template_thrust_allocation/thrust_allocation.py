import numpy as np
import numpy.typing as npt

def B(alpha) -> npt.ArrayLike:
    alpha1, alpha2 = alpha
    return np.array([
        [0, np.cos(alpha1), np.cos(alpha2)],
        [1, np.sin(alpha1), np.sin(alpha2)],
        [l_0, l_1x*np.sin(alpha1) - l_1y*np.cos(alpha1), l_2x*np.sin(alpha2) - l_2y*np.cos(alpha2)]
    ])

# Weighted Moore-Penrose pseudoinverse
# Obtained from lecure notes 5 on "DP control design" - page 15 - 
def weighted_pseudoinverse(B: npt.ArrayLike, W: npt.ArrayLike) -> npt.ArrayLike:
    W_inv = np.linalg.inv(W)  
    B_T = np.transpose(B)

    return W_inv @ B_T @ np.linalg.inv(B @ W_inv @ B_T + epsilon*np.eye(3)) 

# output: Optimal force and angles
# azimuth_angles is unused for this method
def extended(desired_forces: list, azimuth_angles: list, thrust_configuration: list):
    B_pseudo = weighted_pseudoinverse(thrust_configuration, W)
    # Calculate optimal force F*
    F_star = B_pseudo @ tau_cmd + (np.eye(5) - B_pseudo @ thrust_configuration) @ desired_forces
    F_cmd, alpha_cmd = compute_F_command(F_star)
    
    return F_cmd, alpha_cmd

# Thrust configuration matrix B_extended
# Output: optimal force and angles
def bias_mode(desired_forces: list, azimuth_angles: list, thrust_configuration: list):
    B_pseudo = weighted_pseudoinverse(thrust_configuration, W)
    B_inv = np.linalg.pinv(thrust_configuration)
    F_star = B_pseudo @ tau_cmd + (np.eye(3) - B_pseudo @ thrust_configuration) @ desired_forces
    return F_star, azimuth_angles

# Thrust configuration matrix B_extended
# Output: optimal force and angles 
def quad_prog(desired_forces: list, thrus_configuration: list, azimuth_angles: list):
    # TODO: To be implemented based on the matlab code
    F_star = 1
    F_cmd, azimuth_angles = compute_F_command(F_star)
    
    return F_cmd, azimuth_angles

def compute_F_command(F_star): # For extended thrust allocation 
    F0_norm = F_star[0]
    F1_norm = np.sqrt(F_star[1]**2 + F_star[2]**2)
    F2_norm = np.sqrt(F_star[3]**2 + F_star[4]**2)
    alpha_1 = np.arctan2(F_star[2], F_star[1])
    alpha_2 = np.arctan2(F_star[4], F_star[3])
    F_cmd = np.array([F0_norm, F1_norm, F2_norm])
    alpha_cmd = np.array([alpha_1, alpha_2])
    
    return F_cmd, alpha_cmd

def remove_negative_thrust(u_cmd: list, alpha_cmd: list):
    assert len(u_cmd) == len(alpha_cmd)
    for angle in alpha_cmd:
        assert angle < 2*np.pi

    for i in range(1, 3):  # Only for azimuth-thrustere (u1, u2)
        if u_cmd[i] < 0:  # Hvis thrust er negativ, snu vinkelen
            u_cmd[i], alpha_cmd[i] = -u_cmd[i], (alpha_cmd[i] + np.pi)%(2 * np.pi)

    return u_cmd, alpha_cmd

# Configurations
# Thruster position
l_0 = 0.370  # m
l_1x = -0.415
l_1y = -0.07
l_2x = -0.415
l_2y = 0.07

epsilon = 1e-9 # Small value used in weighted pseudoinverse to avoid singularity, from lecture video on pseudoinvere in marine controlsystems 1

# Thrust coeffisient matrix
K = np.diag([2.629, 1.03, 1.03])  # k0, k1, k2

# Azimuth-angle(rad) in Bias-mode
alpha0 = np.pi / 2
alpha1 = np.pi / 2
alpha2 = np.pi / 2
azimuth_angles = np.array([alpha0, alpha1, alpha2])

# Thruster configuration matrix (B-matrix)
B_bias = np.array([
    [0, np.cos(alpha1), np.cos(alpha2)],
    [1, np.sin(alpha1), np.sin(alpha2)],
    [l_0, l_1x*np.sin(alpha1) - l_1y*np.cos(alpha1), l_2x*np.sin(alpha2) - l_2y*np.cos(alpha2)]
])

# Extended Thruster configuration matrix (B-matrix)
B_extended = np.array([
    [0, 1, 0, 1, 0],
    [1, 0, 1, 0, 1],
    [l_0, -l_1y, l_1x, -l_2y, l_2x]
])

# Weight matrix 
W = np.diag([1, 1, 1]) # placeholder
#W_extended = np.diag([5, 1, 1, 1, 1]) # placeholder 

desired_forces = np.array([1, 1, 1]) # placeholder
#desired_forces_extended = np.array([1, 1, 1, 1, 1]) # placeholder

# Thrust command (Body frame)
tau_cmd = np.array([1, -1, -0.5])  # Forces in X, Y and moment N

# Test functions
optimal_force, alpha_cmd = bias_mode(desired_forces, azimuth_angles, B_bias)

# Calculate optimal thruster input u*
u_star = np.linalg.inv(K) @ optimal_force  # Bruk bare de første 3 inputene

u_cmd, alpha_cmd = remove_negative_thrust(u_star, azimuth_angles)

# Results
#print("Optimal thrustkraft F*:", F_star_fin)
print("Optimal thruster-input u*:", u_star)
print("Gyldig thruster-input u_cmd:", u_cmd)
print("Gyldige Azi-vinkler alpha_cmd (radianer):", alpha_cmd)
print("Gyldige Azi-vinkler alpha_cmd (deg):", [angle / np.pi * 180 for angle in alpha_cmd])