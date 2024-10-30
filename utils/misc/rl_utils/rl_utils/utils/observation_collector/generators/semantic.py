__all__ = [
    "PedestrianRelativeLocationGenerator",
    "PedestrianRelativeVelGenerator",
    "PedestrianRelativeVelXGenerator",
    "PedestrianRelativeVelYGenerator",
    "PedestrianDistanceGenerator",
]

from typing import Any, Dict, List, Type

import numpy as np

from ..collectors import (
    BaseUnit,
    PedestrianLocationCollector,
    PedestrianVelXCollector,
    PedestrianVelYCollector,
    RobotPoseCollector,
    PedestrianTypeCollector,
)
from ..utils.semantic import get_relative_pos_to_robot, get_relative_vel_to_robot
from .base_generator import ObservationGeneratorUnit

ObservationDict = Dict[str, Any]


class PedestrianRelativeLocationGenerator(ObservationGeneratorUnit):
    name: str = "ped_relative_location"
    requires: List[BaseUnit] = [RobotPoseCollector, PedestrianLocationCollector]
    data_class: Type[np.ndarray] = np.ndarray

    def generate(
        self,
        obs_dict: ObservationDict,
        *args,
        **kwargs,
    ) -> np.ndarray:
        ped_location: PedestrianLocationCollector.data_class = obs_dict[
            PedestrianLocationCollector.name
        ]

        if len(ped_location.points) == 0:
            return np.array([])

        return get_relative_pos_to_robot(
            robot_pose=obs_dict[RobotPoseCollector.name],
            distant_poses=np.stack(
                [
                    [frame.location.x, frame.location.y, 1]
                    for frame in ped_location.points
                ]
            ),
        )


class PedestrianRelativeVelGenerator(ObservationGeneratorUnit):
    name: str = "ped_relative_vel"
    requires: List[BaseUnit] = [
        RobotPoseCollector,
        PedestrianVelXCollector,
        PedestrianVelYCollector,
    ]
    data_class: Type[np.ndarray] = np.ndarray

    def generate(
        self,
        obs_dict: ObservationDict,
        *args,
        **kwargs,
    ) -> np.ndarray:
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


class PedestrianRelativeVelXGenerator(ObservationGeneratorUnit):
    name: str = "ped_relative_vel_x"
    requires: List[BaseUnit] = [PedestrianRelativeVelGenerator]
    data_class: Type[np.ndarray] = np.ndarray

    def generate(
        self,
        obs_dict: ObservationDict,
        *args,
        **kwargs,
    ) -> np.ndarray:
        ped_rel_vel: PedestrianRelativeVelGenerator.data_class = obs_dict[
            PedestrianRelativeVelGenerator.name
        ]
        return ped_rel_vel[:, 0] if len(ped_rel_vel) > 0 else ped_rel_vel


class PedestrianRelativeVelYGenerator(ObservationGeneratorUnit):
    name: str = "ped_relative_vel_y"
    requires: List[BaseUnit] = [PedestrianRelativeVelGenerator]
    data_class: Type[np.ndarray] = np.ndarray

    def generate(
        self,
        obs_dict: ObservationDict,
        *args,
        **kwargs,
    ) -> np.ndarray:
        ped_rel_vel: PedestrianRelativeVelGenerator.data_class = obs_dict[
            PedestrianRelativeVelGenerator.name
        ]

        return ped_rel_vel[:, 1] if len(ped_rel_vel) > 0 else ped_rel_vel


class PedestrianDistanceGenerator(ObservationGeneratorUnit):
    name: str = "ped_type_distances"
    requires: List[BaseUnit] = [
        PedestrianRelativeLocationGenerator,
        PedestrianTypeCollector,
    ]
    data_class: Dict[int, float] = {}

    def generate(
        self,
        obs_dict: "ObservationDict",
        *args,
        **kwargs,
    ) -> np.ndarray:
        ped_distances = {}

        relative_locations = obs_dict.get(
            PedestrianRelativeLocationGenerator.name, None
        )
        pedestrian_types = obs_dict.get(PedestrianTypeCollector.name, None)

        if relative_locations is None or pedestrian_types is None:
            return ped_distances

        if len(relative_locations) == 0 or len(pedestrian_types.points) == 0:
            return ped_distances

        distances = np.linalg.norm(relative_locations, axis=1)
        types = np.array(
            [int(type_data.evidence) for type_data in pedestrian_types.points]
        )

        # get the unique types
        for _type in np.unique(types):
            ped_distances[_type] = np.min(distances[types == _type])

        return ped_distances
