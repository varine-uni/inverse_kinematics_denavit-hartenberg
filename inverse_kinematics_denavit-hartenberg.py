import os
import time
import pybullet as p
import numpy as np
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
    
# Forward kinematic function, inputs 3 angles and 2 segment lengths
def forward_kinematics(theta1, theta2, theta3, l1, l2):
    # Define DH parameters
    alpha = [np.pi/2, 0, 0]
    a = [0, l1, l2]
    d = [0, l1, 0]

    # Create matrices with parameters
    A1_0 = dh_transform_matrix(theta1, alpha[0], a[0], d[0])
    A2_1 = dh_transform_matrix(theta2, alpha[1], a[1], d[1])
    A3_2 = dh_transform_matrix(theta3, alpha[2], a[2], d[2])
    
    # Dot product of the matrices to get the rotational matrix and translation vector
    t_end_effector_1 = A1_0
    t_end_effector_2 = np.dot(t_end_effector_1, A2_1)
    t_end_effector_3 = np.dot(t_end_effector_2, A3_2)
    
    # Extract positions from the matrices
    x1, y1, z1 = t_end_effector_1[0, 3], t_end_effector_1[1, 3], t_end_effector_1[2, 3]
    x2, y2, z2 = t_end_effector_2[0, 3], t_end_effector_2[1, 3], t_end_effector_2[2, 3]
    x3, y3, z3 = t_end_effector_3[0, 3], t_end_effector_3[1, 3], t_end_effector_3[2, 3]
    
    return x1, y1, z1, x2, y2, z2, x3, y3, z3

# This code defines an optimization objective function to minimize the squared errors between the end-effector position and targeted position
def inverse_kinematics(target_x, target_y, target_z, l1, l2):
    def objective(theta):
        x1, y1, z1, x2, y2, z2, x3, y3, z3 = forward_kinematics(theta[0], theta[1], theta[2], l1, l2)
        error_x = (x1 - target_x)**2 + (x2 - target_x)**2 + (x3 - target_x)**2
        error_y = (y1 - target_y)**2 + (y2 - target_y)**2 + (y3 - target_y)**2
        error_z = (z1 - target_z)**2 + (z2 - target_z)**2 + (z3 - target_z)**2
        return error_x + error_y + error_z
        
    result = minimize(objective, [0, 0, 0], method="BFGS")
    
    if result.success:
        joint_angles = result.x
        return joint_angles
    else:
        raise ValueError("Inverse kinematics optimization failure")
    
# Connect to the PyBullet physics server
p.connect(p.GUI)
p.setGravity(0, 0, -98)  # Set gravity along the negative Z-axis

# Create a flat plane
plane_id = p.createCollisionShape(p.GEOM_PLANE)
plane_body_id = p.createMultiBody(0, plane_id)

# Get the absolute path to the URDF file in the same folder as the script
script_folder = os.path.dirname(os.path.abspath(__file__))
robot_urdf_path = os.path.join(script_folder, "basic_arm_bot.urdf")

# Load the robot URDF
robot_start_pos = [0, 0, 0]  # Adjust the starting position as needed
robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])  # Adjust the orientation as needed
robot_id = p.loadURDF(robot_urdf_path, basePosition=robot_start_pos, baseOrientation=robot_start_orientation)


# Set up the camera
p.resetDebugVisualizerCamera(cameraDistance=4.5, cameraYaw=-90, cameraPitch=-10, cameraTargetPosition=[0, 0, 0])

# Simulation loop
while True:
    target_x = 2
    target_y = 2
    target_z = 1
    
    l1 = 1
    l2 = 1
    
    # Calculate joint angles
    joint_angles = inverse_kinematics(target_x, target_y, target_z, l1, l2)
    
    # Set joint angles for the robot
    for i, joint_angle in enumerate(joint_angles):
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_angle)
    
    p.stepSimulation()
    time.sleep(1. / 240.)