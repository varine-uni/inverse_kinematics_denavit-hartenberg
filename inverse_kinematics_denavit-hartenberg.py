import numpy as np
from mpl_toolkits import mplot3d
from scipy.optimize import minimize

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
    A1_0 = dh_transform_matrix(theta1, alpha[0], a[0], d[0])
    A2_1 = dh_transform_matrix(theta2, alpha[1], a[1], d[1])
    A3_2 = dh_transform_matrix(theta3, alpha[2], a[2], d[2])

    # Dot product of the matrices to get the rotational matrix and translation vector
    t_end_effector = np.dot(np.dot(A1_0, A2_1), A3_2)
    
    # Extract positions from the matrix
    x = t_end_effector[0, 3]
    y = t_end_effector[1, 3]
    z = t_end_effector[2, 3]
    
    return x, y, z

def inverse_kinematics(target_x, target_y, target_z, l1, l2, l3):
    def objective(theta):
        x, y, z = forward_kinematics(theta[0], theta[1], theta[2], l1, l2, l3)
        return (x - target_x)**2 + (y - target_y)**2 + (z - target_z)**2
    
    initial_angles = [0, 0, 0]
    
    result = minimize(objective, initial_angles, method="BFGS")
    
    if result.success:
        joint_angles = result.x
        return joint_angles
    else:
        raise ValueError("Inverse kinematics optimization failure")
    
# Test
target_x = 2
target_y = 1
target_z = 3
    
l1 = 2
l2 = 2
l3 = 2
    
joint_angles = inverse_kinematics(target_x, target_y, target_z, l1, l2, l3)
    
print("Joint Angles:", joint_angles)