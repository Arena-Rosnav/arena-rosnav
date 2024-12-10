
import os
import typing

import nav_msgs.msg
import nav2_msgs.srv
import numpy as np
import rclpy
import rclpy.callback_groups
import rclpy.client

from task_generator.constants import Constants
from task_generator.utils.time import Time

from .utils import WorldMap
from .world_manager import WorldManager

_DUMMY_MAP_SHAPE = (200, 200)
_DUMMY_MAP = nav_msgs.msg.OccupancyGrid(
    info=nav_msgs.msg.MapMetaData(
        height=_DUMMY_MAP_SHAPE[0],
        width=_DUMMY_MAP_SHAPE[1],
        resolution=0.1,
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

    first_world: bool

    def _world_callback(self, value: typing.Any) -> bool:

        if not self.first_world and \
                (simulator := self.node.conf.Arena.SIMULATOR.value) in (Constants.Simulator.GAZEBO,):
            raise RuntimeError(
                f'Simulator {simulator.value} does not support world reloading.')

        self.first_world = False

        world_name = str(value)
        self.node.get_logger().warn(f'LOADING WORLD {value}')
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
        if self._world.map.time < costmap.info.map_load_time:
            self.update_world(WorldMap.from_costmap(costmap))

    def _setup_world_callbacks(self):

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

        # retrieving map from map_server
        self.node.create_subscription(
            nav_msgs.msg.OccupancyGrid,
            'map',
            self._map_callback,
            1,
        )

    def __init__(self) -> None:
        WorldManager.__init__(self, WorldMap.from_costmap(_DUMMY_MAP))
        self.first_world = True
        self._setup_world_callbacks()
