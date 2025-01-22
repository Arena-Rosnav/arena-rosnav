import dataclasses
import json
import os
import yaml
from typing import List
import math
import random

from task_generator.constants import Constants
from task_generator.shared import DynamicObstacle, Obstacle, ModelWrapper
from task_generator.tasks.obstacles import Obstacles, TM_Obstacles
from task_generator.utils.ros_params import ROSParam

from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union
import numpy as np

from rcl_interfaces.msg import SetParametersResult
from task_generator.utils.arena import get_simulation_setup_path


@dataclasses.dataclass
class _ParsedConfig:
    static: List[Obstacle]
    dynamic: List[DynamicObstacle]


class TM_Environment(TM_Obstacles):

    _config: ROSParam[_ParsedConfig]

    def _create_rooms_from_walls(self) -> List[Polygon]:
        """
        Example helper that uses self._PROPS.world_manager.walls
        to construct Shapely polygons representing "rooms."

        Depending on how walls are stored, you may have to:
        1. Convert each set of walls to line segments or a closed polygon
        2. Possibly unify them or keep them separate if there are multiple rooms
        """
        # For demonstration, let’s assume walls are something like
        #   [{"points": [(x1, y1), (x2, y2), ... ]}, ...]
        # and each item defines a closed polygon. Adapt as needed.
        polygons = []
        for wall in self._PROPS.world_manager.walls:
            poly = Polygon(wall["points"])
            if poly.is_valid and not poly.is_empty:
                polygons.append(poly)
        return polygons

    def _is_region_free(
        self,
        occupancy_grid: np.ndarray,
        world_x: float,
        world_y: float,
        width: float,
        height: float,
        margin: float = 0.0,
        rotation_deg: float = 0.0,
    ) -> bool:
        """
        Checks if the rectangular region of size (width x height) with some margin
        at center (world_x, world_y) is free in the occupancy grid.

        This method requires converting from (world_x, world_y)
        to the grid’s cell coordinates. Then check that every cell is empty.

        This is a *placeholder* that you must adapt to your actual transformation
        from world coordinates to grid indices. Typically, you might have:
        grid_x = int((world_x - origin_x) / resolution)
        grid_y = int((world_y - origin_y) / resolution)
        and so on.

        You also need to rotate the bounding box around (world_x, world_y)
        if you want to handle group rotation at the occupancy-grid level.
        """
        # For demonstration, let's assume the grid’s origin is (0,0)
        # and resolution=1.0 (obviously adapt in real code).
        # We do no rotation in this stub, but you can incorporate it.

        # Convert center to grid coords
        grid_cx = int(world_x)  # stub, adapt as needed
        grid_cy = int(world_y)  # stub, adapt as needed

        # Expand half extents
        half_w = int(math.ceil((width + margin) / 2.0))
        half_h = int(math.ceil((height + margin) / 2.0))

        # Build slices
        min_x = grid_cx - half_w
        max_x = grid_cx + half_w
        min_y = grid_cy - half_h
        max_y = grid_cy + half_h

        # Check boundaries
        if (min_y < 0 or min_x < 0 or
            max_y >= occupancy_grid.shape[0] or
                max_x >= occupancy_grid.shape[1]):
            return False

        region = occupancy_grid[min_y:max_y, min_x:max_x]

        # Suppose "0" means free (EMPTY) and "1" means occupied or unknown.
        # Adjust logic as needed.
        return bool((region == 0).all())

    def _mark_region_occupied(
        self,
        occupancy_grid: np.ndarray,
        world_x: float,
        world_y: float,
        width: float,
        height: float,
        margin: float = 0.0,
        rotation_deg: float = 0.0,
    ):
        """
        Example method to mark the placed group’s footprint as “occupied”
        in your occupancy_grid. Very similar to _is_region_free but sets the array.

        For brevity, we’ll assume no rotation in indexing,
        but you could do a more advanced approach.
        """
        grid_cx = int(world_x)
        grid_cy = int(world_y)

        half_w = int(math.ceil((width + margin) / 2.0))
        half_h = int(math.ceil((height + margin) / 2.0))

        min_x = grid_cx - half_w
        max_x = grid_cx + half_w
        min_y = grid_cy - half_h
        max_y = grid_cy + half_h

        if (min_x < 0 or min_y < 0 or
            max_x >= occupancy_grid.shape[1] or
                max_y >= occupancy_grid.shape[0]):
            return  # out of bounds, ignore

        occupancy_grid[min_y:max_y, min_x:max_x] = 1  # Mark all as occupied

    def _parse_environment(self, environment_file: str) -> _ParsedConfig:

        environment_path = os.path.join(
            get_simulation_setup_path(),
            'configs',
            'environment',
            environment_file
        )

        with open(environment_path) as f:
            environment = yaml.safe_load(f)
            print(environment)

        obstacles: List[Obstacle] = []
        static_obstacles = [
            Obstacle.parse(
                obs,
                model=self._PROPS.model_loader.bind(obs["model"])
            )
            for obs in environment.get("obstacles", {}).get("static", [])
            + environment.get("obstacles", {}).get("interactive", [])
        ]

        dynamic_obstacles = [
            DynamicObstacle.parse(
                obs,
                model=self._PROPS.dynamic_model_loader.bind(
                    obs.get("model", Constants.DEFAULT_PEDESTRIAN_MODEL))
            )
            for obs in environment.get("obstacles", {}).get("dynamic", [])
        ]
        rooms = self._create_rooms_from_walls()
        if not rooms:
            print("[WARNING] No rooms found! (check your walls data)")
            # return _ParsedConfig(
            #     static=static_obstacles,
            #     dynamic=dynamic_obstacles
            # )
        for i in range(len(environment["groups"])):
            group = self.node.conf.General.RNG.value.choice(
                environment["groups"]
            )
            print(group)
            group_name = group["name"]
            group_size = group["size"]    # [width, height]
            margin = group.get("margin", 0.5)  # Extra margin
            # Possible angles: 0°, 60°, 90°, 120°, 180°, ...
            possible_rotations = [0, 60, 90, 120, 180, 240, 270, 300]
            # For demonstration, pick a single rotation for the entire group:
            rotation_deg = random.choice(possible_rotations)

            # Occupancy grid
            occupancy_grid = self._PROPS.world_manager.world.map.occupancy.grid

            # We do a naive tile-based approach: scan each room from bottom-left
            # to top-right in some stride to see if we can place the group.
            # This is just an example. You can do more advanced sampling or logic.
            n_groups = 0
            for idx, room_polygon in enumerate(rooms):
                minx, miny, maxx, maxy = room_polygon.bounds

                # Start from a half-size offset
                step_x = group_size[0] + margin
                step_y = group_size[1] + margin
                x = minx + step_x / 2.0
                y = miny + step_y / 2.0

                while y + step_y / 2.0 <= maxy:
                    x = minx + step_x / 2.0
                    while x + step_x / 2.0 <= maxx:
                        candidate = Point(x, y)
                        if room_polygon.contains(candidate):
                            # Check occupancy
                            if self._is_region_free(
                                occupancy_grid,
                                x,
                                y,
                                group_size[0],
                                group_size[1],
                                margin=margin,
                                rotation_deg=rotation_deg
                            ):
                                group_entities = group.get("entities", [])
                                for j, entity in enumerate(group_entities):
                                    ex_off, ey_off, e_theta = entity["position"]

                                    radians = math.radians(rotation_deg)
                                    rot_x = ex_off * math.cos(radians) - ey_off * math.sin(radians)
                                    rot_y = ex_off * math.sin(radians) + ey_off * math.cos(radians)
                                    rot_theta = e_theta + rotation_deg

                                    obstacle_x = x + rot_x
                                    obstacle_y = y + rot_y

                                    obs_name = f"G_{group_name}_{n_groups}_{entity['model']}_{j}"
                                    new_obstacle = Obstacle.parse(
                                        {
                                            "name": obs_name,
                                            "position": [obstacle_x, obstacle_y, rot_theta],
                                            "model": entity["model"],
                                        },
                                        model=self._PROPS.model_loader.bind(entity["model"])
                                    )
                                    obstacles.append(new_obstacle)

                                # Mark region as occupied
                                # (Here, do the same bounding box -> grid region
                                #  and set occupancy = 1, for instance.)
                                self._mark_region_occupied(
                                    occupancy_grid,
                                    x, y,
                                    group_size[0],
                                    group_size[1],
                                    margin=margin,
                                    rotation_deg=rotation_deg
                                )

                                n_groups += 1
                        x += step_x
                    y += step_y

        return _ParsedConfig(
            static=[
                Obstacle.parse(
                    obs,
                    model=self._PROPS.model_loader.bind(obs["model"])
                )
                for obs
                in
                environment.get("obstacles", {}).get("static", []) +
                environment.get("obstacles", {}).get("interactive", [])
            ],
            dynamic=[
                DynamicObstacle.parse(
                    obs,
                    model=self._PROPS.dynamic_model_loader.bind(
                        obs.get("model", Constants.DEFAULT_PEDESTRIAN_MODEL))
                )
                for obs
                in
                environment.get("obstacles", {}).get("dynamic", [])
            ]
        )

    def reset(self, **kwargs) -> Obstacles:
        return self._config.value.static, self._config.value.dynamic

    def __init__(self, **kwargs):
        TM_Obstacles.__init__(self, **kwargs)

        self._config = self.node.ROSParam[_ParsedConfig](
            self.namespace('file'),
            'default.json',
            parse=self._parse_environment,
        )
