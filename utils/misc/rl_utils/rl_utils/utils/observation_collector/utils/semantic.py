import numpy as np
from geometry_msgs.msg import Pose2D
from typing import List, Tuple
import crowdsim_msgs.msg as crowdsim_msgs


def get_relative_pos_to_robot(
    robot_pose: np.ndarray, distant_poses: np.ndarray
) -> np.ndarray:
    """
    Calculates the relative positions of distant frames to the robot's pose.

    Args:
        robot_pose (np.ndarray): The pose of the robot, represented as a numpy array. Represented as structured array containing x, y, and yaw.
        distant_poses (np.ndarray): The distant frames to calculate the relative positions for, represented as a numpy array.
                                    Shape in (num_distant_frames, 3). Each row contains x-coordinate, y-coordinate, and 1 (homogenous component).

    Returns:
        np.ndarray: The relative positions of the distant frames to the robot's pose, represented as a numpy array.
    """
    # Create the homogeneous transformation matrix map_T_robot
    map_T_robot = np.array(
        [
            [
                np.cos(robot_pose["yaw"]),  # Rotation component: cos(yaw)
                -np.sin(robot_pose["yaw"]),  # Rotation component: -sin(yaw)
                robot_pose["x"],  # Translation component: x
            ],
            [
                np.sin(robot_pose["yaw"]),  # Rotation component: sin(yaw)
                np.cos(robot_pose["yaw"]),  # Rotation component: cos(yaw)
                robot_pose["y"],  # Translation component: y
            ],
            [0, 0, 1],  # Homogeneous component: 0, 0, 1
        ]
    )

    # Calculate the inverse transformation matrix robot_T_map
    robot_T_map = np.linalg.inv(map_T_robot)

    # Apply the transformation to the distant poses using einsum, return the transformed poses, excluding the homogeneous component
    return np.einsum("ij,kj->ki", robot_T_map, distant_poses)[:, :2]


def get_relative_vel_to_robot(
    robot_pose: np.ndarray,
    pedestrian_vel_vector: np.ndarray,
) -> np.ndarray:
    """
    Calculates the relative velocity of pedestrians to the robot.

    Args:
        robot_pose (np.ndarray): The pose of the robot, including its position and orientation.
        pedestrian_vel_vector (np.ndarray): The velocity vectors of pedestrians in (num_peds, 2).
                                            Each row contains the x and y components of the velocity.

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
