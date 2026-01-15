import numpy as np
import math

def inverse_kinematics_2d_planar(target_pos, link_lengths):
    """
    Solve inverse kinematics for a 2D planar arm with 2 links

    Args:
        target_pos: [x, y] target position for end effector
        link_lengths: [L1, L2] lengths of the two links

    Returns:
        joint_angles: [theta1, theta2] joint angles to reach target
        or None if target is unreachable
    """
    x, y = target_pos
    L1, L2 = link_lengths

    # Calculate distance from origin to target
    r = math.sqrt(x**2 + y**2)

    # Check if target is reachable
    if r > L1 + L2:
        print("Target is out of reach")
        return None
    if r < abs(L1 - L2):
        print("Target is inside the workspace")
        return None

    # Calculate theta2 using law of cosines
    cos_theta2 = (L1**2 + L2**2 - r**2) / (2 * L1 * L2)
    # Clamp to avoid numerical errors
    cos_theta2 = max(-1, min(1, cos_theta2))
    theta2 = math.acos(cos_theta2)

    # Calculate intermediate angles
    alpha = math.atan2(y, x)
    beta = math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))

    # Calculate theta1
    theta1 = alpha - beta

    return [theta1, theta2]

def inverse_kinematics_2d_planar_both_solutions(target_pos, link_lengths):
    """
    Solve inverse kinematics for both possible solutions

    Args:
        target_pos: [x, y] target position for end effector
        link_lengths: [L1, L2] lengths of the two links

    Returns:
        solutions: List of possible [theta1, theta2] solutions
    """
    x, y = target_pos
    L1, L2 = link_lengths

    # Calculate distance from origin to target
    r = math.sqrt(x**2 + y**2)

    # Check if target is reachable
    if r > L1 + L2:
        print("Target is out of reach")
        return []
    if r < abs(L1 - L2):
        print("Target is inside the workspace")
        return []

    # Calculate theta2 using law of cosines (two possible values)
    cos_theta2 = (L1**2 + L2**2 - r**2) / (2 * L1 * L2)
    # Clamp to avoid numerical errors
    cos_theta2 = max(-1, min(1, cos_theta2))
    theta2 = math.acos(cos_theta2)

    solutions = []

    # Solution 1: positive theta2
    alpha = math.atan2(y, x)
    beta = math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
    theta1 = alpha - beta
    solutions.append([theta1, theta2])

    # Solution 2: negative theta2
    theta2_neg = -theta2
    beta_neg = math.atan2(L2 * math.sin(theta2_neg), L1 + L2 * math.cos(theta2_neg))
    theta1_neg = alpha - beta_neg
    solutions.append([theta1_neg, theta2_neg])

    return solutions

def jacobian_inverse_kinematics(current_angles, target_pos, link_lengths,
                                max_iterations=100, tolerance=1e-6):
    """
    Solve inverse kinematics using Jacobian transpose method

    Args:
        current_angles: Starting joint angles
        target_pos: Target end-effector position
        link_lengths: Link lengths
        max_iterations: Maximum number of iterations
        tolerance: Position tolerance for convergence

    Returns:
        joint_angles: Solution that gets close to target position
    """
    angles = np.array(current_angles, dtype=float)

    for i in range(max_iterations):
        # Calculate current end-effector position
        current_pos, _ = forward_kinematics_2d(angles.tolist(), link_lengths)

        # Calculate error
        error = np.array(target_pos) - current_pos

        # Check if we're close enough
        if np.linalg.norm(error) < tolerance:
            print(f"Converged after {i+1} iterations")
            return angles.tolist()

        # Calculate Jacobian
        jacobian = calculate_jacobian_2d(angles.tolist(), link_lengths)

        # Update angles using Jacobian transpose method
        # Note: This is a simplified version; in practice, you'd want to adjust the step size
        angles = angles + 0.01 * jacobian.T @ error

    print(f"Did not converge after {max_iterations} iterations")
    return angles.tolist()

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

def rotation_matrix_2d(theta):
    """
    Create a 2D rotation matrix for angle theta (in radians)
    """
    return np.array([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta), math.cos(theta)]
    ])

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
    link_lengths = [1.0, 0.8]

    # Target position
    target_pos = [1.2, 0.8]

    print("Target position:", target_pos)
    print("Link lengths:", link_lengths)

    # Solve inverse kinematics analytically
    solution = inverse_kinematics_2d_planar(target_pos, link_lengths)
    if solution:
        print("Analytical solution (theta1, theta2):", solution)

        # Verify by calculating forward kinematics
        fk_pos, _ = forward_kinematics_2d(solution, link_lengths)
        print("Verification - Forward kinematics result:", fk_pos)
        print("Error:", np.linalg.norm(np.array(target_pos) - fk_pos))

    # Get both solutions
    both_solutions = inverse_kinematics_2d_planar_both_solutions(target_pos, link_lengths)
    print("\nBoth possible solutions:")
    for i, sol in enumerate(both_solutions):
        print(f"Solution {i+1}: {sol}")

        # Verify by calculating forward kinematics
        fk_pos, _ = forward_kinematics_2d(sol, link_lengths)
        print(f"Verification: {fk_pos}, Error: {np.linalg.norm(np.array(target_pos) - fk_pos)}")

    # Solve using Jacobian method
    print("\nSolving with Jacobian method (starting from [0, 0]):")
    jacobian_solution = jacobian_inverse_kinematics([0, 0], target_pos, link_lengths)
    print("Jacobian solution (theta1, theta2):", jacobian_solution)

    # Verify by calculating forward kinematics
    fk_pos, _ = forward_kinematics_2d(jacobian_solution, link_lengths)
    print("Verification - Forward kinematics result:", fk_pos)
    print("Error:", np.linalg.norm(np.array(target_pos) - fk_pos))