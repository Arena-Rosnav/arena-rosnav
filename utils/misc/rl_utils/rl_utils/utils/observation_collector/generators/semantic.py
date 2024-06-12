from typing import Any, Dict, List, Type

import numpy as np

from ..collectors import (
    BaseUnit,
    PedestrianLocationCollector,
    PedestrianVelXCollector,
    PedestrianVelYCollector,
    RobotPoseCollector,
)
from ..utils.semantic import get_relative_pos_to_robot, get_relative_vel_to_robot
from .base_generator import ObservationGeneratorUnit

ObservationDict = Dict[str, Any]

__all__ = [
    "PedestrianRelativeLocation",
    "PedestrianRelativeVel",
    "PedestrianRelativeVelX",
    "PedestrianRelativeVelY",
]


class PedestrianRelativeLocation(ObservationGeneratorUnit):
    name: str = "ped_relative_location"
    requires: List[BaseUnit] = [RobotPoseCollector, PedestrianLocationCollector]
    data_class: Type[np.ndarray] = np.ndarray

    def generate(self, obs_dict: ObservationDict) -> np.ndarray:
        ped_location: PedestrianLocationCollector.data_class = obs_dict[
            PedestrianLocationCollector.name
        ]

        if len(ped_location.points) == 0:
            return np.array([])

        return get_relative_pos_to_robot(
            robot_pose=obs_dict[RobotPoseCollector.name],
            distant_frames=np.stack(
                [
                    [frame.location.x, frame.location.y, 1]
                    for frame in ped_location.points
                ]
            ),
        )


class PedestrianRelativeVel(ObservationGeneratorUnit):
    name: str = "ped_relative_vel"
    requires: List[BaseUnit] = [
        RobotPoseCollector,
        PedestrianVelXCollector,
        PedestrianVelYCollector,
    ]
    data_class: Type[np.ndarray] = np.ndarray

    def generate(self, obs_dict: ObservationDict) -> np.ndarray:
        ped_vel_x: PedestrianVelXCollector.data_class = obs_dict[
            PedestrianVelXCollector.name
        ]
        ped_vel_y: PedestrianVelYCollector.data_class = obs_dict[
            PedestrianVelYCollector.name
        ]

        if len(ped_vel_x.points) == 0 or len(ped_vel_y.points) == 0:
            return np.array([])

        ped_vel = np.stack(
            [
                [data_x.evidence, data_y.evidence]
                for data_x, data_y in zip(ped_vel_x.points, ped_vel_y.points)
            ]
        )

        return get_relative_vel_to_robot(
            robot_pose=obs_dict[RobotPoseCollector.name],
            pedestrian_vel_vector=ped_vel,
        )


class PedestrianRelativeVelX(ObservationGeneratorUnit):
    name: str = "ped_relative_vel_x"
    requires: List[BaseUnit] = [PedestrianRelativeVel]
    data_class: Type[np.ndarray] = np.ndarray

    def generate(self, obs_dict: ObservationDict) -> np.ndarray:
        ped_rel_vel: PedestrianRelativeVel.data_class = obs_dict[
            PedestrianRelativeVel.name
        ]
        return ped_rel_vel[0, :] if len(ped_rel_vel) > 0 else ped_rel_vel


class PedestrianRelativeVelY(ObservationGeneratorUnit):
    name: str = "ped_relative_vel_y"
    requires: List[BaseUnit] = [PedestrianRelativeVel]
    data_class: Type[np.ndarray] = np.ndarray

    def generate(self, obs_dict: ObservationDict) -> np.ndarray:
        ped_rel_vel: PedestrianRelativeVel.data_class = obs_dict[
            PedestrianRelativeVel.name
        ]

        return ped_rel_vel[1, :] if len(ped_rel_vel) > 0 else ped_rel_vel
