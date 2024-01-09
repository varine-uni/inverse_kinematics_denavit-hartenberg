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
    
# Define DH parameters
alpha = [0, 0, 0]
a = [2, 0, 2]
d = [0, 0, 0]
theta = [np.pi*2, np.pi, np.pi]

# Create matrices with parameters
A1_0 = dh_transform_matrix(theta[0], alpha[0], a[0], d[0])
A2_1 = dh_transform_matrix(theta[1], alpha[1], a[1], d[1])
A3_2 = dh_transform_matrix(theta[2], alpha[2], a[2], d[2])