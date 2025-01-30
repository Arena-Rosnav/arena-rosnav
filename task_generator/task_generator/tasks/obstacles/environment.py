import dataclasses
import itertools
import json
import os
import shapely
import yaml
from typing import List, Dict
import math
import random

from task_generator.constants import Constants
from task_generator.shared import DynamicObstacle, Obstacle, ModelWrapper, PositionOrientation
from task_generator.tasks.obstacles import Obstacles, TM_Obstacles
from task_generator.utils.ros_params import ROSParam

from shapely.geometry import Point, Polygon, box
from shapely.geometry import LineString, Polygon, MultiPolygon
from shapely.ops import unary_union, polygonize
from collections import deque
from scipy.ndimage import label
import numpy as np

from rcl_interfaces.msg import SetParametersResult
from task_generator.utils.arena import get_simulation_setup_path
from collections import defaultdict


@dataclasses.dataclass
class _ParsedConfig:
    static: List[Obstacle]
    dynamic: List[DynamicObstacle]


class TM_Environment(TM_Obstacles):

    _config: ROSParam[_ParsedConfig]

    def calculate_world_bounds(self):
        all_walls = list(self._PROPS.world_manager.walls) + list(self._PROPS.world_manager.detected_walls)
        x_min = y_min = np.inf
        x_max = y_max = -np.inf

        for wall in all_walls:
            x1, y1 = wall.Start.x, wall.Start.y
            x2, y2 = wall.End.x, wall.End.y
            x_min = min(x_min, x1, x2)
            x_max = max(x_max, x1, x2)
            y_min = min(y_min, y1, y2)
            y_max = max(y_max, y1, y2)

        return x_min, x_max, y_min, y_max

    def merge_walls(self, door_threshold=2.0):
        # Convert door_threshold to float if list is passed
        if isinstance(door_threshold, list):
            door_threshold = float(door_threshold[0]) if door_threshold else 2.0
        else:
            door_threshold = float(door_threshold)

        all_walls = list(self._PROPS.world_manager.walls) + list(self._PROPS.world_manager.detected_walls)
        horizontal, vertical = [], []

        # Classify walls
        for wall in all_walls:
            start = (wall.Start.x, wall.Start.y)
            end = (wall.End.x, wall.End.y)
            if start[1] == end[1]:  # Horizontal
                y = start[1]
                x1, x2 = sorted([start[0], end[0]])
                horizontal.append((y, x1, x2))
            elif start[0] == end[0]:  # Vertical
                x = start[0]
                y1, y2 = sorted([start[1], end[1]])
                vertical.append((x, y1, y2))

        # Merge horizontal walls
        merged_h = []
        walls_by_y = {}
        for y, x1, x2 in horizontal:
            walls_by_y.setdefault(y, []).append((x1, x2))
        for y, segments in walls_by_y.items():
            segments.sort()
            merged = []
            for seg in segments:
                if not merged:
                    merged.append(seg)
                else:
                    last = merged[-1]
                    if seg[0] - last[1] <= door_threshold:
                        merged[-1] = (last[0], max(last[1], seg[1]))
                    else:
                        merged.append(seg)
            merged_h.extend([(y, x1, x2) for x1, x2 in merged])

        # Merge vertical walls
        merged_v = []
        walls_by_x = {}
        for x, y1, y2 in vertical:
            walls_by_x.setdefault(x, []).append((y1, y2))
        for x, segments in walls_by_x.items():
            segments.sort()
            merged = []
            for seg in segments:
                if not merged:
                    merged.append(seg)
                else:
                    last = merged[-1]
                    if seg[0] - last[1] <= door_threshold:
                        merged[-1] = (last[0], max(last[1], seg[1]))
                    else:
                        merged.append(seg)
            merged_v.extend([(x, y1, y2) for y1, y2 in merged])

        return merged_h, merged_v

    def find_matching_vertical(self, vertical_index, x, y):
        return [(vy1, vy2) for (vy1, vy2) in vertical_index.get(x, [])
                if vy1 <= y <= vy2]

    def has_matching_top_wall(self, horizontal_index, x1, x2, top_y):
        return any(x1 <= wx1 and wx2 <= x2
                   for wx1, wx2 in horizontal_index.get(top_y, []))

    def find_room_polygons(self, horizontal, vertical, door_threshold):
        vertical_index = defaultdict(list)
        for x, y1, y2 in vertical:
            vertical_index[x].append((min(y1, y2), max(y1, y2)))

        horizontal_index = defaultdict(list)
        for y, x1, x2 in horizontal:
            horizontal_index[y].append((min(x1, x2), max(x1, x2)))

        rooms = []
        processed = set()

        # Bottom-up room detection with vertical continuity check
        for bottom_y in sorted(horizontal_index.keys()):
            for x1, x2 in horizontal_index[bottom_y]:
                # Find all possible top walls
                for top_y in [y for y in horizontal_index.keys() if y > bottom_y]:
                    for tx1, tx2 in horizontal_index[top_y]:
                        if tx1 <= x1 and tx2 >= x2:
                            # Check vertical continuity with door gap tolerance
                            left_ok = self.check_vertical_coverage(
                                vertical_index.get(x1, []),
                                bottom_y,
                                top_y,
                                door_threshold
                            )
                            right_ok = self.check_vertical_coverage(
                                vertical_index.get(x2, []),
                                bottom_y,
                                top_y,
                                door_threshold
                            )

                            if left_ok and right_ok:
                                room = [
                                    (x1, bottom_y), (x2, bottom_y),
                                    (x2, top_y), (x1, top_y)
                                ]
                                if not self.is_duplicate(room, rooms):
                                    rooms.append(room)
                                    processed.update({
                                        (x1, bottom_y, x2, bottom_y),
                                        (x2, bottom_y, x2, top_y),
                                        (x2, top_y, x1, top_y),
                                        (x1, top_y, x1, bottom_y)
                                    })

        return rooms

    def check_vertical_coverage(self, segments, y_start, y_end, max_gap):
        segments = sorted(segments, key=lambda x: x[0])
        coverage_start = y_start
        coverage_end = y_start

        for seg in segments:
            seg_start, seg_end = seg
            if seg_start > coverage_end + max_gap:
                return False  # Gap too large

            coverage_end = max(coverage_end, seg_end)
            if coverage_end >= y_end:
                return True

        return coverage_end >= y_end

    def is_duplicate(self, new_room, existing_rooms):
        new_points = sorted(new_room)
        for room in existing_rooms:
            if sorted(room) == new_points:
                return True
        return False

    def filter_world_rooms(self, rooms, x_min, x_max, y_min, y_max):
        filtered = []
        world_corners = {(x_min, y_min), (x_max, y_min),
                         (x_max, y_max), (x_min, y_max)}

        for room in rooms:
            room_corners = set(room)
            if not room_corners.issuperset(world_corners):
                filtered.append(room)
        return filtered

    def filter_world_bounds(self, rooms, x_min, x_max, y_min, y_max):
        filtered = []
        world_edges = {
            (x_min, y_min), (x_max, y_min),
            (x_max, y_max), (x_min, y_max)
        }

        for room in rooms:
            room_edges = set(room)
            if not room_edges.issuperset(world_edges):
                filtered.append(room)

        return filtered

    def filter_rooms(self, rooms):
        filtered = []
        for room in rooms:
            points = np.array(room)
            dx = max(points[:, 0]) - min(points[:, 0])
            dy = max(points[:, 1]) - min(points[:, 1])

            if dx > 0 and dy > 0 and (dx * dy) > 10:
                filtered.append(room)
        return filtered

    def _is_room_inside(self, room_a, room_b):
        # Check if room_a is entirely inside room_b
        a_points = np.array(room_a)
        b_points = np.array(room_b)
        return (np.all(a_points[:, 0] >= b_points[:, 0].min()) and
                np.all(a_points[:, 0] <= b_points[:, 0].max()) and
                np.all(a_points[:, 1] >= b_points[:, 1].min()) and
                np.all(a_points[:, 1] <= b_points[:, 1].max()))

    def _create_rooms_from_walls(self, door_threshold=4.0):  # Increased threshold for example scenario
        x_min, x_max, y_min, y_max = self.calculate_world_bounds()
        merged_h, merged_v = self.merge_walls(door_threshold)
        rooms = self.find_room_polygons(merged_h, merged_v, door_threshold)
        return self.filter_world_rooms(rooms, x_min, x_max, y_min, y_max)

    def visualize_rooms(self, walls, rooms, bounds=None):

        import matplotlib as mpl
        import matplotlib.pyplot as plt
        """Visualize walls and detected rooms"""
        fig, ax = plt.subplots(figsize=(10, 10))

        # Draw walls
        for wall in walls:
            start = (wall.Start.x, wall.Start.y)
            end = (wall.End.x, wall.End.y)
            ax.plot([start[0], end[0]], [start[1], end[1]], 'k-', linewidth=2)

        # Draw rooms with transparency
        for i, room in enumerate(rooms):
            polygon = mpl.patches.Polygon(room, closed=True, alpha=0.3,
                                          edgecolor='blue', facecolor=f'C{i%10}')
            ax.add_patch(polygon)
            # Add room number annotation
            center_x = sum(p[0] for p in room) / 4
            center_y = sum(p[1] for p in room) / 4
            ax.text(center_x, center_y, str(i + 1),
                    ha='center', va='center', fontsize=8)

        # Set plot limits using bounds if provided
        if bounds:
            ax.set_xlim(bounds['x_min'] - 2, bounds['x_max'] + 2)
            ax.set_ylim(bounds['y_min'] - 2, bounds['y_max'] + 2)
        else:
            ax.autoscale()

        ax.set_aspect('equal')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Room Detection Visualization')
        plt.grid(True)
        plt.savefig('test.png')

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
            self.node.get_logger().info("Environment:")
            print(environment)

        static_obstacles: List[Obstacle] = []
        dynamic_obstacles: List[DynamicObstacle] = []

        if (zones := self.node._world_manager.zones):
            rooms = [zone.polygon for zone in zones]
        else:
            walls = list(self._PROPS.world_manager.walls) + list(self._PROPS.world_manager.detected_walls)
            # print(walls)
            rooms = [shapely.Polygon(room) for room in self._create_rooms_from_walls()]
            # if not rooms:
            # print("[WARNING] No rooms found! (check your walls data)")
            # return _ParsedConfig(
            #     static=static_obstacles,
            #     dynamic=dynamic_obstacles
            # )
        print(len(rooms))
        print(rooms)

        groups = environment["groups"]
        offset = random.randint(0, len(groups))
        groups_selection = list(itertools.islice(itertools.cycle(groups), offset, offset + len(rooms)))
        random.shuffle(groups_selection)
        groups_iter = iter(groups_selection)

        # self.visualize_rooms(walls, rooms)
        for idx, room_polygon in enumerate(rooms):
            group = next(groups_iter)
            print(group)
            group_name = group["name"]
            group_size = group["size"]    # [width, height]
            margin = group.get("margin", 0.5)  # Extra margin

            # Occupancy grid
            occupancy_grid = self._PROPS.world_manager.world.map.occupancy.grid

            # We do a naive tile-based approach: scan each room from bottom-left
            # to top-right in some stride to see if we can place the group.
            # This is just an example. You can do more advanced sampling or logic.
            n_groups = 0
            minx, miny, max_x, max_y = room_polygon.bounds

            # Start from a half-size offset
            step_x = group_size[0] + margin
            step_y = group_size[1] + margin
            x = minx + step_x / 2.0
            y = miny + step_y / 2.0

            while y + step_y / 2.0 <= max_y:
                x = minx + step_x / 2.0
                while x + step_x / 2.0 <= max_x:
                    candidate = Point(x, y)
                    if room_polygon.contains(candidate):
                        # Check occupancy
                        if True or self._is_region_free(
                            occupancy_grid,
                            x,
                            y,
                            group_size[0],
                            group_size[1],
                            margin=margin,
                            rotation_deg=0,
                        ):
                            # Possible angles: 0°, 60°, 90°, 120°, 180°, ...
                            rotation_deg = self.node.conf.General.RNG.value.choice(group.get('rotations', [0]))

                            group_static_entities = group.get("entities", {}).get('static', [])
                            group_dynamic_entites = group.get('entities', {}).get('dynamic', [])
                            for j, entity in enumerate(group_static_entities):
                                ex_off, ey_off, e_theta = entity["position"]

                                radians = math.radians(rotation_deg)
                                rot_x = ex_off * math.cos(radians) - ey_off * math.sin(radians)
                                rot_y = ex_off * math.sin(radians) + ey_off * math.cos(radians)
                                rot_theta = math.fmod(e_theta + rotation_deg, 360) / 180 * math.pi

                                obstacle_x = x + rot_x
                                obstacle_y = y + rot_y

                                obs_name = f"G_{group_name}_{n_groups}_{entity['model']}_{j}"
                                new_obstacle = Obstacle(
                                    name=obs_name,
                                    position=PositionOrientation(x=obstacle_x, y=obstacle_y, orientation=rot_theta),
                                    model=self._PROPS.model_loader.bind(entity["model"]),
                                    extra={},
                                )
                                static_obstacles.append(new_obstacle)
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
        print(static_obstacles)
        return _ParsedConfig(
            static=static_obstacles,
            dynamic=[
                # DynamicObstacle.parse(
                #     obs,
                #     model=self._PROPS.dynamic_model_loader.bind(
                #         obs.get("model", Constants.DEFAULT_PEDESTRIAN_MODEL))
                # )
                # for obs
                # in
                # environment.get("obstacles", {}).get("dynamic", [])
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
