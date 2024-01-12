import numpy as np
from scipy.optimize import minimize
#import pybullet as p

# Example using 3DOF

# The Denavit-Hartenberg transformation matrix
def dh_transform_matrix(theta, alpha, a, d):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    
# Forward kinematic function, inputs 3 angles and 2 segment lengths
def forward_kinematics(theta1, theta2, theta3, l1, l2):
    # Define DH parameters
    alpha = [0, np.pi/2, 0]
    a = [0, l1, l2]
    d = [0, 0, 0]

    # Create matrices with parameters
    A1_0 = dh_transform_matrix(theta1, alpha[0], a[0], d[0])
    A2_1 = dh_transform_matrix(theta2, alpha[1], a[1], d[1])
    A3_2 = dh_transform_matrix(theta3, alpha[2], a[2], d[2])

    #print("A1:", A1_0)
    #print("A2:", A2_1)
    #print("A3:", A3_2)
    
    # Dot product of the matrices to get the rotational matrix and translation vector
    t_end_effector_1 = A1_0
    t_end_effector_2 = np.dot(t_end_effector_1, A2_1)
    t_end_effector_3 = np.dot(t_end_effector_2, A3_2)
    
    #print(t_end_effector_1)
    #print(t_end_effector_2)
    #print(t_end_effector_3)
    
    # Extract positions from the matrices
    x1, y1, z1 = t_end_effector_1[0, 3], t_end_effector_1[1, 3], t_end_effector_1[2, 3]
    x2, y2, z2 = t_end_effector_2[0, 3], t_end_effector_2[1, 3], t_end_effector_2[2, 3]
    x3, y3, z3 = t_end_effector_3[0, 3], t_end_effector_3[1, 3], t_end_effector_3[2, 3]
    
    return x1, y1, z1, x2, y2, z2, x3, y3, z3

def inverse_kinematics(target_x, target_y, target_z, l1, l2):
    def objective(theta):
        x1, y1, z1, x2, y2, z2, x3, y3, z3 = forward_kinematics(theta[0], theta[1], theta[2], l1, l2)
        return (x3 - target_x)**2 + (y3 - target_y)**2 + (z3 - target_z)**2
        
    result = minimize(objective, [0, 0, 0], method="BFGS")
    
    if result.success:
        joint_angles = result.x
        return joint_angles
    else:
        raise ValueError("Inverse kinematics optimization failure")
    
# Test
target_x = 2
target_y = 2
target_z = 3
    
l1 = 3
l2 = 3
    
joint_angles = inverse_kinematics(target_x, target_y, target_z, l1, l2)
    
print("Joint Angles:", joint_angles)

# Calculate forward kinematics
x1, y1, z1, x2, y2, z2, x3, y3, z3 = forward_kinematics(joint_angles[0], joint_angles[1], joint_angles[2], l1, l2)

# End effector final coords
print("x:", x3, "y:", y3, "z:", z3)
    