import numpy as np
import math

def rotation_matrix_2d(theta):
    """
    Create a 2D rotation matrix for angle theta (in radians)
    """
    return np.array([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta), math.cos(theta)]
    ])

def forward_kinematics_2d(joint_angles, link_lengths):
    """
    Calculate the end-effector position for a 2D planar arm

    Args:
        joint_angles: List of joint angles [theta1, theta2, ...]
        link_lengths: List of link lengths [L1, L2, ...]

    Returns:
        end_effector_pos: [x, y] position of the end effector
        joint_positions: List of [x, y] positions for each joint
    """
    if len(joint_angles) != len(link_lengths):
        raise ValueError("Number of joint angles must match number of link lengths")

    # Starting position at origin
    current_pos = np.array([0.0, 0.0])
    joint_positions = [current_pos.copy()]

    # Calculate rotation matrix for first joint
    total_angle = joint_angles[0]
    rot_matrix = rotation_matrix_2d(total_angle)

    # For each link, calculate the next joint position
    for i in range(len(link_lengths)):
        # Calculate the vector from current position to next joint
        link_vector = np.array([link_lengths[i], 0.0])

        # Rotate the link vector by the total angle up to this point
        rotated_vector = rot_matrix @ link_vector

        # Calculate the next joint position
        next_pos = current_pos + rotated_vector
        joint_positions.append(next_pos.copy())

        # Update current position
        current_pos = next_pos.copy()

        # If not the last link, update the total rotation for the next link
        if i < len(link_lengths) - 1:
            total_angle += joint_angles[i + 1]
            rot_matrix = rotation_matrix_2d(total_angle)

    end_effector_pos = current_pos

    return end_effector_pos, joint_positions

def calculate_jacobian_2d(joint_angles, link_lengths):
    """
    Calculate the Jacobian matrix for a 2D planar arm
    The Jacobian relates joint velocities to end-effector velocities
    """
    n = len(joint_angles)
    jacobian = np.zeros((2, n))  # 2D position, n joints

    # Calculate cumulative angles
    cumulative_angles = np.zeros(n)
    cumulative_angles[0] = joint_angles[0]
    for i in range(1, n):
        cumulative_angles[i] = cumulative_angles[i-1] + joint_angles[i]

    # Calculate end-effector position using forward kinematics
    end_pos, _ = forward_kinematics_2d(joint_angles, link_lengths)

    # Calculate Jacobian elements
    for i in range(n):
        # Position of the i-th joint
        if i == 0:
            joint_pos = np.array([0.0, 0.0])
        else:
            # Calculate position of i-th joint
            temp_pos, _ = forward_kinematics_2d(joint_angles[:i], link_lengths[:i])
            joint_pos = temp_pos

        # Calculate Jacobian column for i-th joint
        jacobian[0, i] = -(end_pos[1] - joint_pos[1])  # -dy
        jacobian[1, i] = (end_pos[0] - joint_pos[0])   # dx

    return jacobian

# Example usage
if __name__ == "__main__":
    # Example: 2-link planar arm
    joint_angles = [math.pi/4, math.pi/6]  # 45 degrees, 30 degrees
    link_lengths = [1.0, 0.8]  # Link lengths

    end_pos, joint_positions = forward_kinematics_2d(joint_angles, link_lengths)

    print("Joint angles (radians):", joint_angles)
    print("Link lengths:", link_lengths)
    print("End-effector position:", end_pos)
    print("Joint positions:", joint_positions)

    # Calculate Jacobian
    jacobian = calculate_jacobian_2d(joint_angles, link_lengths)
    print("Jacobian matrix:")
    print(jacobian)