import numpy as np
from scipy.linalg import block_diag, solve_continuous_are
from scipy.linalg import solve_continuous_lyapunov

# Physical Parameters of the Quadcopter
MASS = 1.6  # kg (including Raspberry Pi, Navio2, and battery)
GRAVITY = 9.81  # m/s^2
ARM_LENGTH = 0.225  # meters (distance from center to motor)
Ixx = 0.0347  # kg.m^2 (moment of inertia around x-axis)
Iyy = 0.0458  # kg.m^2 (moment of inertia around y-axis)
Izz = 0.0977  # kg.m^2 (moment of inertia around z-axis)

# State Space Representation
# State Vector: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]^T
# Control Input Vector: [thrust, tau_phi, tau_theta, tau_psi]^T

# State Transition Matrix A (12x12)
A = np.array([[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, -GRAVITY, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, GRAVITY, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

# Input Matrix B (12x4)
B = np.array([[0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [1/MASS, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 1/Ixx, 0, 0],  # Tau_phi influences p
              [0, 0, 1/Iyy, 0],  # Tau_theta influences q
              [0, 0, 0, 1/Izz]]) # Tau_psi influences r

# Output Matrix C
C = np.eye(12)

# Direct Transmission Matrix D (No direct feedthrough)
D = np.zeros((6, 4))

# Print the state space matrices
print("State Transition Matrix A:\n", A)
print("Input Matrix B:\n", B)
print("Output Matrix C:\n", C)
print("Direct Transmission Matrix D:\n", D)

# Save matrices to files for future use
np.save('A_matrix.npy', A)
np.save('B_matrix.npy', B)
np.save('C_matrix.npy', C)
np.save('D_matrix.npy', D)

# LQR Gain Calculation
# Define cost matrices Q and R
Q = np.diag([100, 100, 100, 10, 10, 10, 100, 100, 100, 10, 10, 10])  # Penalizes state deviations
R = np.diag([1, 1, 1, 1])   # Penalizes control effort

# Solve the continuous-time Algebraic Riccati Equation (CARE) to find P
P = solve_continuous_are(A, B, Q, R)

# Calculate the LQR gain matrix K
K = np.linalg.inv(R) @ B.T @ P

# Extracting the reduced K matrix for implementation (3x3)
# We are interested in controlling z, phi, and theta using thrust, tau_phi, and tau_theta respectively.
K_reduced = K[[0, 1, 2], :][:, [2, 6, 7]]

# Print the LQR gain matrix
print("LQR Gain Matrix K (Full):\n", K)
print("Reduced LQR Gain Matrix K (3x3):\n", K_reduced)

# Save the LQR gain matrices to file
np.save('K_matrix.npy', K)
np.save('K_reduced_matrix.npy', K_reduced)

# Kalman Filter Gain Calculation
# Define process noise covariance (W) and measurement noise covariance (V)
W = np.eye(12) * 1.0  # Increased process noise for stability
V = np.eye(12) * 1.0   # Increased measurement noise for stability

# Solve the continuous-time Algebraic Riccati Equation for the observer
try:
    P_obs = solve_continuous_are(A.T, C.T, W, V)
    # Calculate the Kalman gain matrix L
    L = P_obs @ C.T @ np.linalg.inv(V)
    # Print the Kalman gain matrix
    print("Kalman Gain Matrix L:\n", L)
    # Save the Kalman gain matrix to file
    np.save('L_matrix.npy', L)
except np.linalg.LinAlgError:
    print("Failed to find a finite solution for the Kalman gain matrix L.")

# Observability Check
n = A.shape[0]  # Number of states
observability_matrix = C
for i in range(1, n):
    observability_matrix = np.vstack((observability_matrix, C @ np.linalg.matrix_power(A, i)))

rank_observability = np.linalg.matrix_rank(observability_matrix)
print("Rank of Observability Matrix:", rank_observability)
if rank_observability == n:
    print("The system is observable.")
else:
    print("The system is not fully observable.")

# Notes on Analysis
# The A matrix represents the system dynamics, capturing how position, velocity, orientation, and angular rates evolve over time.
# The B matrix represents how control inputs (thrust and torques) affect these states.
# The C matrix represents the measured states, focusing on velocity and angular velocity.
# The D matrix is zero since there is no direct feedthrough from inputs to outputs.
# The LQR gain matrix K is computed to provide optimal state feedback control, balancing between minimizing state error and control effort.
# The reduced gain matrix K_reduced is extracted to control only altitude (z), roll (phi), and pitch (theta).
# The Kalman gain matrix L is computed to optimally estimate the state vector using the available measurements and the system model. If a finite solution is not found, consider adjusting W and V for better stability.
