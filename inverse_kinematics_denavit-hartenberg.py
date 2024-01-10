import numpy as np
from mpl_toolkits import mplot3d

# Example using 3DOF

# The Denavit-Hartenberg transformation matrix
def dh_transform_matrix(theta, alpha, a, d):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    
# Forward kinematic function, inputs 3 angles and 3 segment lengths
def forward_kinematics(theta1, theta2, theta3, l1, l2, l3):
    # Define DH parameters
    alpha = [0, 0, 0]
    a = [l1, l2, l3]
    d = [0, 0, 0]

    # Create matrices with parameters
    A1_0 = dh_transform_matrix(theta1[0], alpha[0], a[0], d[0])
    A2_1 = dh_transform_matrix(theta2[1], alpha[1], a[1], d[1])
    A3_2 = dh_transform_matrix(theta3[2], alpha[2], a[2], d[2])

    # Dot product of the matrices to get the rotational matrix and translation vector
    t_end_effector = np.dot(np.dot(A1_0, A2_1), A3_2)
    
    # Extract positions from the matrix
    x = t_end_effector[0, 3]
    y = t_end_effector[1, 3]
    z = t_end_effector[2, 3]
    
    return x, y, z

# def inverse_kinematics(target_x, target_y, target_z, l1, l2, l3):
    