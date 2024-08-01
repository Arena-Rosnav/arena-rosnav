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
    robot_pose: np.ndarray,
    pedestrian_vel_vector: np.ndarray,
) -> np.ndarray:
    """
    Calculates the relative velocity of pedestrians to the robot.

    Args:
        robot_pose (np.ndarray): The pose of the robot, including its position and orientation.
        pedestrian_vel_vector (np.ndarray): The velocity vectors of pedestrians in (num_peds, 2 [x-vel, y-vel]).

    Returns:
        np.ndarray: The relative velocity vectors of pedestrians with respect to the robot
                    in the robot's coordinate frame, shape (num_peds, 2).
    """
    map_R_robot = np.array(
        [
            [np.cos(robot_pose["yaw"]), -np.sin(robot_pose["yaw"])],
            [np.sin(robot_pose["yaw"]), np.cos(robot_pose["yaw"])],
        ]
    )
    robot_R_map = np.linalg.inv(map_R_robot)

    if len(pedestrian_vel_vector) > 0:
        rel_vel = np.matmul(robot_R_map, pedestrian_vel_vector.T).T
        return rel_vel
    return np.empty((0, 2))
