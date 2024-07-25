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


def get_relative_vel_to_robot(
    robot_pose: Pose2D,
    pedestrian_vel_vector: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Calculates the relative velocity of pedestrians to the robot.

    Args:
        robot_pose (Pose2D): The pose of the robot.
        semantic_data_x (List[crowdsim_msgs.SemanticDatum]): List of semantic data along the x-axis.
        semantic_data_y (List[crowdsim_msgs.SemanticDatum]): List of semantic data along the y-axis.

    Returns:
        Tuple[np.ndarray, np.ndarray]: A tuple containing the relative velocities along the x-axis and y-axis.
    """
    map_R_robot = np.array(
        [
            [np.cos(robot_pose["yaw"]), -np.sin(robot_pose["yaw"])],
            [np.sin(robot_pose["yaw"]), np.cos(robot_pose["yaw"])],
        ]
    )
    robot_R_map = np.linalg.inv(map_R_robot)

    if len(pedestrian_vel_vector) > 0:
        rel_vel = np.matmul(robot_R_map, pedestrian_vel_vector.T)
        return rel_vel
    return np.array([])
