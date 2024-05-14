from typing import Any, Dict, List, Tuple

import numpy as np

from .aggregate_collector_unit import AggregateCollectorUnit
import crowdsim_msgs.msg as crowdsim_msgs
from geometry_msgs.msg import Point, Pose2D
from crowdsim_agents.utils import SemanticAttribute
from rl_utils.utils.observation_collector.constants import OBS_DICT_KEYS


class SemanticAggregateUnit(AggregateCollectorUnit):
    @staticmethod
    def get_relative_pos_to_robot(
        robot_pose: Pose2D, distant_frames: np.ndarray
    ) -> np.ndarray:
        """
        Calculates the relative positions of distant pedestrians to the robot.

        Args:
            robot_pose (Pose2D): The pose of the robot.
            distant_frames (crowdsim_msgs.SemanticData): The semantic data of distant pedestrians.

        Returns:
            np.ndarray: The relative positions of distant pedestrians to the robot.
        """
        robot_pose_array = np.array([robot_pose.x, robot_pose.y, robot_pose.theta])

        # homogeneous transformation matrix: map_T_robot
        map_T_robot = np.array(
            [
                [
                    np.cos(robot_pose_array[2]),
                    -np.sin(robot_pose_array[2]),
                    robot_pose_array[0],
                ],
                [
                    np.sin(robot_pose_array[2]),
                    np.cos(robot_pose_array[2]),
                    robot_pose_array[1],
                ],
                [0, 0, 1],
            ]
        )

        robot_T_map = np.linalg.inv(map_T_robot)

        return np.einsum("ij,kj->ki", robot_T_map, distant_frames)[:, :2]

    @staticmethod
    def get_relative_vel_to_robot(
        robot_pose: Pose2D,
        semantic_data_x: List[crowdsim_msgs.SemanticDatum],
        semantic_data_y: List[crowdsim_msgs.SemanticDatum],
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
        robot_pose_array = np.array([robot_pose.x, robot_pose.y, robot_pose.theta])
        map_R_robot = np.array(
            [
                [np.cos(robot_pose_array[2]), -np.sin(robot_pose_array[2])],
                [np.sin(robot_pose_array[2]), np.cos(robot_pose_array[2])],
            ]
        )
        robot_R_map = np.linalg.inv(map_R_robot)

        # x/y velocity
        if len(semantic_data_x.points) > 0 and len(semantic_data_y.points) > 0:
            ped_vel = np.stack(
                [
                    [data_x.evidence, data_y.evidence]
                    for data_x, data_y in zip(
                        semantic_data_x.points, semantic_data_y.points
                    )
                ]
            )
            rel_vel = np.matmul(robot_R_map, ped_vel.T)
            return rel_vel[0, :].T, rel_vel[1, :].T
        return np.array([]), np.array([])

    def get_observations(
        self, obs_dict: Dict[str, Any], *args, **kwargs
    ) -> Dict[str, Any]:
        obs_dict = super().get_observations(obs_dict, *args, **kwargs)

        if len(obs_dict[SemanticAttribute.IS_PEDESTRIAN.value].points) > 0:
            ped_data = obs_dict[OBS_DICT_KEYS.SEMANTIC.PEDESTRIAN_LOCATION.value].points
            obs_dict[OBS_DICT_KEYS.SEMANTIC.RELATIVE_LOCATION.value] = (
                SemanticAggregateUnit.get_relative_pos_to_robot(
                    robot_pose=obs_dict[OBS_DICT_KEYS.ROBOT_POSE],
                    distant_frames=np.stack(
                        [[frame.location.x, frame.location.y, 1] for frame in ped_data]
                    ),
                )
            )

            (
                obs_dict[OBS_DICT_KEYS.SEMANTIC.RELATIVE_X_VEL.value],
                obs_dict[OBS_DICT_KEYS.SEMANTIC.RELATIVE_Y_VEL.value],
            ) = SemanticAggregateUnit.get_relative_vel_to_robot(
                robot_pose=obs_dict[OBS_DICT_KEYS.ROBOT_POSE],
                semantic_data_x=obs_dict[SemanticAttribute.PEDESTRIAN_VEL_X.value],
                semantic_data_y=obs_dict[SemanticAttribute.PEDESTRIAN_VEL_Y.value],
            )
        else:
            (
                obs_dict[OBS_DICT_KEYS.SEMANTIC.RELATIVE_LOCATION.value],
                obs_dict[OBS_DICT_KEYS.SEMANTIC.RELATIVE_X_VEL.value],
                obs_dict[OBS_DICT_KEYS.SEMANTIC.RELATIVE_Y_VEL.value],
            ) = (None, None, None)

        return obs_dict
