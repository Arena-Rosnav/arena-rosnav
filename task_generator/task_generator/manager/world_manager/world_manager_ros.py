
import os
import typing

import yaml
import nav_msgs.msg
import nav2_msgs.srv
import numpy as np
import rclpy
import rclpy.callback_groups
import rclpy.client

from task_generator.constants import Constants
from task_generator.utils.time import Time
from task_generator.shared import Position, Wall

from .utils import WorldMap, WorldWalls, WorldObstacleConfiguration, WorldObstacleConfigurations, WorldZones, Zone
from .world_manager import WorldManager

_DUMMY_MAP_SHAPE = (200, 200)
_DUMMY_MAP = nav_msgs.msg.OccupancyGrid(
    info=nav_msgs.msg.MapMetaData(
        height=_DUMMY_MAP_SHAPE[0],
        width=_DUMMY_MAP_SHAPE[1],
        resolution=0.1,
        map_load_time=Time(-1, 0).to_time(),
    ),
    data=list(
        np.pad(
            np.zeros(
                (_DUMMY_MAP_SHAPE[0] - 2, _DUMMY_MAP_SHAPE[1] - 2),
                dtype=int,
            ),
            ((1, 1), (1, 1)),
            mode='constant',
            constant_values=1
        ).flat
    )
)


class WorldManagerROS(WorldManager):
    cli: rclpy.client.Client

    _first_world: bool
    _world_name: str
    _callbacks: typing.List[typing.Callable[[], None]]

    @classmethod
    def _load_walls(cls, yaml_path: str) -> WorldWalls | None:
        try:
            with open(yaml_path) as f:
                walls_yaml = yaml.safe_load(f)
            walls: WorldWalls = [
                Wall.parse(wall)
                for wall
                in walls_yaml['walls']
            ]
            return walls
        except Exception:
            return None

    @classmethod
    def _load_obstacles(cls, yaml_path: str) -> WorldObstacleConfigurations | None:
        try:
            with open(yaml_path) as f:
                obstacles_yaml = yaml.safe_load(f)

            obstacles: WorldObstacleConfigurations = [
                WorldObstacleConfiguration.parse(obstacle)
                for obstacle
                in obstacles_yaml['static']
            ]
            return obstacles
        except Exception:
            return None

    @classmethod
    def _load_zones(cls, yaml_path: str) -> WorldZones | None:
        try:
            with open(yaml_path) as f:
                zones_yaml = yaml.safe_load(f)

            zones: WorldZones = [
                Zone.parse(zone)
                for zone
                in zones_yaml
            ]
            return zones
        except Exception:
            return None

    def _world_callback(self, value: typing.Any) -> bool:
        world_name = str(value)

        if world_name != self._world_name and \
                not self._first_world and \
                (simulator := self.node.conf.Arena.SIMULATOR.value) in (Constants.Simulator.GAZEBO,):
            raise RuntimeError(
                f'Simulator {simulator.value} does not support world reloading.')

        self.node.get_logger().warn(f'LOADING WORLD {world_name}')
        self._world_name = world_name
        self._first_world = False

        map_yaml = os.path.join(
            self.node.conf.Arena.get_world_path(world_name),
            'map',
            'map.yaml',
        )
        response = self.cli.call(
            nav2_msgs.srv.LoadMap.Request(
                map_url=f'{map_yaml}'
            )
        )

        if response is None:
            raise RuntimeError(
                f'failed to load map for world {world_name}: service timed out')

        if response.result > 0:
            raise RuntimeError(
                f'failed to load map for world {world_name}: status code {response.result}')

        return True

    def _map_callback(self, costmap: nav_msgs.msg.OccupancyGrid):
        if self._first_world:
            return
        if self._world.map.time < costmap.info.map_load_time:
            obstacles = self._load_obstacles(
                os.path.join(
                    self.node.conf.Arena.get_world_path(self._world_name),
                    'map',
                    'obstacles.yaml',
                )
            )
            walls = self._load_walls(
                os.path.join(
                    self.node.conf.Arena.get_world_path(self._world_name),
                    'map',
                    'walls.yaml',
                )
            )
            zones = self._load_zones(
                os.path.join(
                    self.node.conf.Arena.get_world_path(self._world_name),
                    'map',
                    'zones.yaml',
                )
            )
            self.update_world(
                WorldMap.from_costmap(costmap),
                obstacles=obstacles,
                walls=walls,
                zones=zones,
            )
            for callback in self._callbacks:
                try:
                    callback()
                except Exception:
                    pass

    def _setup_world_callbacks(self):

        # retrieving map from map_server
        self.node.create_subscription(
            nav_msgs.msg.OccupancyGrid,
            '/map',
            self._map_callback,
            1,
        )

        # publishing map to map_server
        self.cli = self.node.create_client(
            nav2_msgs.srv.LoadMap,
            'map_server/load_map',
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),
        )
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('LoadMap service not available, waiting again...')

        self.node.rosparam.callback(
            'world',
            self._world_callback,
        )

    def on_world_change(self, callback: typing.Callable[[], None]):
        self._callbacks.append(callback)

    def __init__(self) -> None:
        WorldManager.__init__(self)
        self._callbacks = []
        self.update_world(world_map=WorldMap.from_costmap(_DUMMY_MAP), obstacles=None, walls=[])
        self._first_world = True
        self._world_name = ''
        self._setup_world_callbacks()
