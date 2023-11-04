import random
from typing import Any, Callable, Collection, Iterator

import numpy as np
import yaml
import xml.etree.ElementTree as ET

from task_generator.constants import FlatlandRandomModel
from task_generator.manager.entity_manager.entity_manager import EntityManager
from task_generator.manager.entity_manager.utils import ObstacleLayer
from task_generator.manager.utils import World
from task_generator.manager.world_manager import WorldManager
from task_generator.shared import DynamicObstacle, Model, ModelType, Obstacle, ObstacleProps
from task_generator.simulators.base_simulator import BaseSimulator

from geometry_msgs.msg import Point

import itertools


class ObstacleManager:

    _world_manager: WorldManager
    _namespace: str
    _entity_manager: EntityManager
    _simulator: BaseSimulator

    id_generator: Iterator[int]

    def __init__(self, namespace, world_manager, simulator: BaseSimulator, entity_manager: EntityManager):
        self._world_manager = world_manager
        self._namespace = namespace
        self._simulator = simulator

        self._entity_manager = entity_manager

        self.id_generator = itertools.count(434)

    def spawn_world_obstacles(self, world: World):
        """
        Loads given obstacles into the simulator,
        the map file is retrieved from launch parameter "map_file"
        """

        self._entity_manager.spawn_line_obstacles(walls=world.walls, heightmap=world.map)
        self._entity_manager.spawn_obstacles(obstacles=world.entities)

    def spawn_dynamic_obstacles(self, setups: Collection[DynamicObstacle]):
        """
        Loads given dynamic obstacles into the simulator.
        To-Do: consider merging with spawn_dynamic_obstacles or simplifying by calling it
        """

        self._entity_manager.spawn_dynamic_obstacles(obstacles=setups)

    def spawn_obstacles(self, setups: Collection[Obstacle]):
        """
        Loads given obstacles into the simulator.
        If the object has an interaction radius of > 0, 
        then load it as an interactive obstacle instead of static
        To-Do: consider merging with spawn_obstacles or simplifying by calling it
        """

        self._entity_manager.spawn_obstacles(obstacles=setups)

    def respawn(self, callback: Callable[[], Any]):
        """
        Unuse obstacles, (re-)use them in callback, finally remove unused obstacles
        @callback: Function to call between unuse and remove
        """
        self._entity_manager.unuse_obstacles()
        callback()
        self._entity_manager.remove_obstacles(purge=ObstacleLayer.UNUSED)

    def reset(self, purge: ObstacleLayer = ObstacleLayer.INUSE):
        """
        Unuse and remove all obstacles
        """
        self._entity_manager.remove_obstacles(purge=purge)

    # TODO refactor this with a registry

    def generate_random_model(self, model_type: ModelType, **kwargs) -> Model:
        if model_type == ModelType.YAML:
            return self.generate_random_model(**kwargs)

        else:
            raise NotImplementedError()

    def _generate_YAML_model(self, is_dynamic=False, min_radius: float = FlatlandRandomModel.MIN_RADIUS, max_radius: float = FlatlandRandomModel.MAX_RADIUS, linear_vel: float = FlatlandRandomModel.LINEAR_VEL, angular_vel_max: float = FlatlandRandomModel.ANGLUAR_VEL_MAX) -> Model:
        """
        Creates a random yaml model.

        Since a lot of the variables are untouched
        the majority of the dict is filled up with
        constants defined in the `Constants` file.
        """

        def generate_random_footprint_type(min_radius: float, max_radius: float) -> dict:
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
            type = random.choice(["circle", "polygon"])

            if type == "circle":
                radius = random.uniform(min_radius, max_radius)

                return {"type": type, "radius": radius}

            # Defined in flatland definition
            points_amount = random.randint(3, 8)
            angle_interval = 2 * np.pi / points_amount

            points = []

            for p in range(points_amount):
                angle = random.uniform(0, angle_interval)
                radius = random.uniform(min_radius, max_radius)

                real_angle = angle_interval * p + angle

                points.append(
                    [np.cos(real_angle) * radius, np.sin(real_angle) * radius]
                )

            return {"type": type, "points": list(points)}

        body = {
            **FlatlandRandomModel.BODY,
            "type": "dynamic" if is_dynamic else "static",
        }

        footprint = {
            **FlatlandRandomModel.FOOTPRINT,
            **generate_random_footprint_type(min_radius, max_radius),
        }

        body["footprints"] = [footprint]

        model = {"bodies": [body], "plugins": []}

        if is_dynamic:
            model["plugins"].append(
                {
                    **FlatlandRandomModel.RANDOM_MOVE_PLUGIN,
                    "linear_velocity": random.uniform(0, linear_vel),
                    "angular_velocity_max": angular_vel_max,
                }
            )

        return Model(
            type=ModelType.YAML,
            name="random_obstacle",
            description=yaml.dump(model),
            path=""
        )
