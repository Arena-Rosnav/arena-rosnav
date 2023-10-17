import random
from typing import Collection, Iterator

import numpy as np
import yaml
import xml.etree.ElementTree as ET

from task_generator.constants import FlatlandRandomModel
from task_generator.manager.dynamic_manager.dynamic_manager import DynamicManager
from task_generator.manager.map_manager import MapManager
from task_generator.shared import DynamicObstacle, Model, ModelType, Obstacle
from task_generator.simulators.base_simulator import BaseSimulator

from geometry_msgs.msg import Point

import itertools


class ObstacleManager:

    _map_manager: MapManager
    _namespace: str
    _dynamic_manager: DynamicManager
    _simulator: BaseSimulator

    first_reset: bool

    id_generator: Iterator[int]

    def __init__(self, namespace, map_manager, simulator: BaseSimulator, dynamic_manager: DynamicManager):
        self._map_manager = map_manager
        self._namespace = namespace
        self._simulator = simulator

        self.first_reset = True
        self._dynamic_manager = dynamic_manager

        self.id_generator = itertools.count(434)

    def spawn_map_obstacles(self, map: ET.ElementTree):
        """
        Loads given obstacles into the simulator,
        the map file is retrieved from launch parameter "map_file"
        """

        root = map.getroot()

        for child in root:

            _from = Point(float(child.attrib['x1']), float(
                child.attrib['y1']), 0)
            _to = Point(float(child.attrib['x2']),
                        float(child.attrib['y2']), 0)

            identifier = str(next(self.id_generator))

            self._dynamic_manager.spawn_line_obstacle(
                name=f"wall{identifier}",
                _from=_from,
                _to=_to
            )

    def spawn_dynamic_obstacles(self, setups: Collection[DynamicObstacle]):
        """
        Loads given dynamic obstacles into the simulator.
        To-Do: consider merging with spawn_dynamic_obstacles or simplifying by calling it
        """

        self._dynamic_manager.spawn_dynamic_obstacles(obstacles=setups)

    def spawn_obstacles(self, setups: Collection[Obstacle]):
        """
        Loads given obstacles into the simulator.
        If the object has an interaction radius of > 0, 
        then load it as an interactive obstacle instead of static
        To-Do: consider merging with spawn_obstacles or simplifying by calling it
        """

        self._dynamic_manager.spawn_obstacles(obstacles=setups)

    def reset(self):

        if self.first_reset:
            self.first_reset = False
        else:
            self._dynamic_manager.remove_obstacles()

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
