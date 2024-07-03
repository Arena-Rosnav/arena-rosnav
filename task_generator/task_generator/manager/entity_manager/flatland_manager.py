from typing import Collection
import rospy
import yaml
from task_generator.constants import Config, FlatlandRandomModel
from task_generator.shared import (
    DynamicObstacle,
    ModelType,
    ModelWrapper,
    Namespace,
    Obstacle,
    ObstacleProps,
    PositionOrientation,
    Robot,
)
from task_generator.simulators.flatland_simulator import FlatlandSimulator
from .entity_manager import EntityManager

from task_generator.shared import Model

import numpy as np
import dataclasses

STATIC_OBS_BASENAME = "static_obs_"
DYNAMIC_OBS_BASENAME = "dynamic_obs_"


class FlatlandManager(EntityManager):

    def __init__(self, namespace: Namespace, simulator: FlatlandSimulator):
        super().__init__(namespace, simulator)
        if not isinstance(simulator, FlatlandSimulator):
            raise TypeError(
                f"Wrong Simulator Type for this EntityManager. (Given: {simulator.__class__}, Expected: {FlatlandSimulator.__class__})"
            )
        self._static_obs_count = 0
        self._dynamic_obs_count = 0

    def spawn_obstacle(self, obstacle: ObstacleProps):
        if not self._simulator.spawn_entity(obstacle):
            rospy.logwarn(f"Couldn't spawn obstacle '{obstacle.name}'")

    def spawn_obstacles(self, obstacles: Collection[Obstacle]):
        for obstacle in obstacles:
            obs_name = FlatlandManager._generate_name(
                is_dynamic=False, count=self._static_obs_count
            )
            obstacle = dataclasses.replace(
                obstacle,
                model=ModelWrapper.from_model(
                    FlatlandManager._generate_YAML_model(obs_name)
                ),
            )
            obstacle = dataclasses.replace(
                obstacle,
                name=FlatlandManager._generate_name(
                    is_dynamic=False, count=self._static_obs_count
                ),
            )

            self.spawn_obstacle(obstacle)

            self._static_obs_count += 1
            self._spawned_obstacles.append(obs_name)

    def spawn_dynamic_obstacles(self, obstacles: Collection[DynamicObstacle]):
        for obstacle in obstacles:
            obs_name = FlatlandManager._generate_name(
                is_dynamic=True, count=self._dynamic_obs_count
            )
            obstacle = dataclasses.replace(
                obstacle,
                model=ModelWrapper.from_model(
                    FlatlandManager._generate_YAML_model(
                        name=obs_name, is_dynamic=True)
                ),
            )
            obstacle = dataclasses.replace(
                obstacle,
                name=obs_name,
            )

            self.spawn_obstacle(obstacle)

            self._dynamic_obs_count += 1
            self._spawned_obstacles.append(obs_name)

    def remove_obstacles(self, purge: bool = True):
        return

    def unuse_obstacles(self):
        # for i in range(self._static_obs_count):
        #     self._simulator.delete_entity(
        #         FlatlandManager._generate_name(is_dynamic=False, count=i)
        #     )

        # for i in range(self._dynamic_obs_count):
        #     self._simulator.delete_entity(
        #         FlatlandManager._generate_name(is_dynamic=True, count=i)
        #     )

        # TODO change all spawns/moves/deletes in base sim to multi-requests
        if not FlatlandSimulator.delete_all_entities(self._simulator, self._spawned_obstacles):  # type: ignore # nopep8
            rospy.logwarn("Couldn't remove obstacles")

        self._spawned_obstacles = []
        self._static_obs_count, self._dynamic_obs_count = 0, 0

    def spawn_robot(self, robot: Robot):
        self._simulator.spawn_entity(robot)

    def move_robot(self, name: str, position: PositionOrientation):
        self._simulator.move_entity(name, position)

    @staticmethod
    def _generate_YAML_model(
        name: str,
        is_dynamic=False,
        min_radius: float = FlatlandRandomModel.MIN_RADIUS,
        max_radius: float = FlatlandRandomModel.MAX_RADIUS,
        linear_vel: float = FlatlandRandomModel.LINEAR_VEL,
        angular_vel_max: float = FlatlandRandomModel.ANGLUAR_VEL_MAX,
    ) -> Model:
        """
        Creates a random yaml model.

        Since a lot of the variables are untouched
        the majority of the dict is filled up with
        constants defined in the `Constants` file.
        """

        def generate_footprint_type(
            is_dynamic: bool, min_radius: float, max_radius: float
        ) -> dict:
            """
            An object in flatland can either be a circle with a
            specific radius or a polygon shape.

            This function will choose a shape randomly and
            creates a shape from this.

            For the circle the radius is chosen randomly and
            lies in a specific range defined in the `constants` file

            For the polygon, the amount of vertexes is determined
            at first. Then the vertexes are distributed around the center
            and for each vertex a distance to the center is calculated.
            At the end, the vertexes form the polygon. The distance
            to the center is chosen randomly and lies in the range
            defined in `constants`.
            """
            _type = "circle" if is_dynamic else "polygon"
            # type = random.choice(["circle", "polygon"])

            if _type == "circle":
                radius = Config.General.RNG.uniform(min_radius, max_radius)

                return {"type": _type, "radius": radius}

            # Defined in flatland definition
            points_amount = Config.General.RNG.integers(3, 8, endpoint=True)
            angle_interval = 2 * np.pi / points_amount

            points = []

            for p in range(points_amount):
                angle = Config.General.RNG.uniform(0, angle_interval)
                radius = Config.General.RNG.uniform(min_radius, max_radius)

                real_angle = angle_interval * p + angle

                points.append(
                    [
                        float(np.cos(real_angle) * radius),
                        float(np.sin(real_angle) * radius),
                    ]
                )

            return {"type": _type, "points": list(points)}

        body = {
            **FlatlandRandomModel.BODY,
            "type": "dynamic" if is_dynamic else "static",
        }

        footprint = {
            **FlatlandRandomModel.FOOTPRINT,
            **generate_footprint_type(is_dynamic, min_radius, max_radius),
        }

        body["footprints"] = [footprint]

        model = {"bodies": [body], "plugins": []}

        if is_dynamic:
            model["plugins"].append(
                {
                    **FlatlandRandomModel.RANDOM_MOVE_PLUGIN,
                    "linear_velocity": Config.General.RNG.uniform(0, linear_vel),
                    "angular_velocity_max": angular_vel_max,
                }
            )

        return Model(
            type=ModelType.YAML,
            name=name,
            description=yaml.dump(model),
            path="",
        )

    @staticmethod
    def _generate_name(is_dynamic: bool, count: int) -> str:
        return (
            f"{DYNAMIC_OBS_BASENAME}{count}"
            if is_dynamic
            else f"{STATIC_OBS_BASENAME}{count}"
        )
