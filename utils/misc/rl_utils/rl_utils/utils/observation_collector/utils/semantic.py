import numpy as np
from geometry_msgs.msg import Pose2D
from typing import List, Tuple
import crowdsim_msgs.msg as crowdsim_msgs


def get_relative_pos_to_robot(
    robot_pose: np.ndarray, distant_frames: np.ndarray
) -> np.ndarray:
    """
    Calculates the relative positions of distant pedestrians to the robot.

    Args:
        robot_pose (np.ndarray): The pose of the robot.
        distant_frames (crowdsim_msgs.SemanticData): The semantic data of distant pedestrians.

    Returns:
        np.ndarray: The relative positions of distant pedestrians to the robot.
    """
    # homogeneous transformation matrix: map_T_robot
    map_T_robot = np.array(
        [
            [
                np.cos(robot_pose["yaw"]),
                -np.sin(robot_pose["yaw"]),
                robot_pose["x"],
            ],
            [
                np.sin(robot_pose["yaw"]),
                np.cos(robot_pose["yaw"]),
                robot_pose["y"],
            ],
            [0, 0, 1],
        ]
    )
    robot_T_map = np.linalg.inv(map_T_robot)
    return np.einsum("ij,kj->ki", robot_T_map, distant_frames)[:, :2]


# def calculate_relative_positions(robot_pose, pedestrian_positions):
#     """
#     Calculates the relative positions of multiple pedestrians with respect to a robot.

#     Args:
#         robot_pose (np.ndarray): The structured array containing robot position and yaw.
#         pedestrian_positions (np.ndarray): An array of (x, y) positions of the pedestrians.

#     Returns:
#         np.ndarray: An array of relative coordinates of the pedestrians in the robot's frame of reference.
#     """
#     # Calculate the relative positions
#     relative_positions = pedestrian_positions - np.array(
#         [robot_pose["x"], robot_pose["y"]]
#     )

#     cos_yaw = np.cos(robot_pose["yaw"])
#     sin_yaw = np.sin(robot_pose["yaw"])

#     # Rotate the relative positions using vectorized operations
#     rotated_positions_x = (
#         cos_yaw * relative_positions[:, 0] + sin_yaw * relative_positions[:, 1]
#     )
#     rotated_positions_y = (
#         -sin_yaw * relative_positions[:, 0] + cos_yaw * relative_positions[:, 1]
#     )

#     return np.column_stack((rotated_positions_x, rotated_positions_y))


def get_relative_vel_to_robot(
    robot_pose: np.ndarray,
    pedestrian_vel_vector: np.ndarray,
) -> np.ndarray:
    """
    Calculates the relative velocity of pedestrians to the robot.

    Args:
        robot_pose (np.ndarray): The pose of the robot, including its position and orientation.
        pedestrian_vel_vector (np.ndarray): The velocity vectors of pedestrians
                                            in (num_peds, 2 [x-vel, y-vel]).

    Returns:
        np.ndarray: The relative velocity vectors of pedestrians with respect to the robot
                    in the robot's coordinate frame, shape (num_peds, 2).
    """
    # Create the rotation matrix to transform from map frame to robot frame
    map_r_robot = np.array(
        [
            [np.cos(robot_pose["yaw"]), -np.sin(robot_pose["yaw"])],
            [np.sin(robot_pose["yaw"]), np.cos(robot_pose["yaw"])],
        ]
    )

    # Use the transpose for transformation
    robot_r_map = map_r_robot.T

    if len(pedestrian_vel_vector) > 0:
        # Transform pedestrian velocities to the robot's coordinate frame
        rel_vel = np.matmul(robot_r_map, pedestrian_vel_vector.T).T
        return rel_vel

    return np.empty((0, 2))
