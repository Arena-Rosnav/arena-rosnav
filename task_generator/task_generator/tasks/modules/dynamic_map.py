import os
from typing import Dict

from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.service import Service
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import SetParametersResult

from task_generator import NodeInterface
from task_generator.constants import Constants
from task_generator.manager.entity_manager.utils import ObstacleLayer
from task_generator.manager.utils import WorldMap
from task_generator.task_generator.shared import rosparam_set
from task_generator.tasks.modules import TM_Module

# Assuming map_distance_server messages are available in ROS 2
from map_distance_server.srv import GetDistanceMap

# DYNAMIC MAP INTERFACE

DynamicMapConfiguration = Dict[str, Dict]

# Define MAP_GENERATOR_NS as a function if it's not available


def MAP_GENERATOR_NS(param_name: str) -> str:
    return f"/map_generator/{param_name}"


class Mod_DynamicMap(TM_Module, NodeInterface):
    def __init__(self, **kwargs):
        NodeInterface.__init__(self)
        TM_Module.__init__(self, **kwargs)

        self.callback_group = ReentrantCallbackGroup()

        self.__map_request_pub = self.node.create_publisher(
            String, self.TOPIC_REQUEST_MAP, 1)
        self.__task_reset_pub = self.node.create_publisher(
            String, self.TOPIC_RESET, 1)

        self.node.create_subscription(
            String,
            self.TOPIC_RESET,
            self._cb_task_reset,
            1,
            callback_group=self.callback_group)

        self.__get_dist_map_service = self.node.create_client(
            GetDistanceMap, self.SERVICE_DISTANCE_MAP, callback_group=self.callback_group)

        self.node.declare_parameter('num_envs', 1)
        self.node.declare_parameter(MAP_GENERATOR_NS('episode_per_map'), 1)

        num_envs = self.node.get_parameter(
            'num_envs').value if "eval_sim" not in self._TASK.robot_managers[0].namespace else 1
        self._target_eps_num = self.node.get_parameter(
            MAP_GENERATOR_NS('episode_per_map')).value * num_envs

        self._episodes = 0
        self.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, params):
        for param in params:
            if param.name == self.PARAM_EPISODES:
                self._episodes = param.value
        return SetParametersResult(successful=True)

    def before_reset(self, **kwargs):
        self._episodes += 1
        if self._episodes >= self._target_eps_num:
            self.request_new_map()

    def _set_config(self, config: DynamicMapConfiguration):
        generator = self.node.get_parameter(self.PARAM_GENERATOR).value
        log = f"Setting [Map Generator: {generator}] parameters"
        config_generator = config.get(generator, dict())
        for key, value in config_generator.items():
            log += f"\t{key}={value}"
            rosparam_set(f"{self.PARAM_GENERATOR_CONFIGS}.{
                         generator}.{key}", value)
        self.node.get_logger().info(log)

    def _cb_task_reset(self, msg):
        self._update_map()

    def _update_map(self):
        future = self.__get_dist_map_service.call_async(
            GetDistanceMap.Request())
        future.add_done_callback(self._update_map_callback)

    def _update_map_callback(self, future):
        try:
            dist_map = future.result()
            self._TASK.world_manager.update_world(
                world_map=WorldMap.from_occupancy_grid(occupancy_grid=dist_map)
            )
            self._TASK.obstacle_manager.reset(purge=ObstacleLayer.WORLD)
            self._TASK.obstacle_manager.spawn_world_obstacles(
                self._TASK.world_manager.world
            )
        except Exception as e:
            self.node.get_logger().warn(f"Service call failed {e}")

    def request_new_map(self):
        self._episodes = 0
        self.__map_request_pub.publish(String())

        try:
            self.node.create_subscription(
                OccupancyGrid,
                self.TOPIC_MAP,
                lambda _: None,
                1,
                callback_group=self.callback_group
            )
            self.node.create_subscription(
                String,
                self.TOPIC_SIGNAL_MAP,
                lambda _: None,
                1,
                callback_group=self.callback_group
            )
        except Exception as e:
            self.node.get_logger().warn(
                f"[Map Generator] Timeout while waiting for new map. Continue with current map. {e}")
        else:
            self.node.get_logger().info("===================")
            self.node.get_logger().info("+++ Got new map +++")
            self.node.get_logger().info("===================")

        self.__task_reset_pub.publish(String())

    PARAM_MAP_FILE = "map_file"
    PARAM_EPISODES = "dynamic_map.curr_eps"
    PARAM_GENERATOR = "generator"
    PARAM_GENERATOR_CONFIGS = "generator_configs"

    TOPIC_REQUEST_MAP = "/request_new_map"
    TOPIC_RESET = "/dynamic_map/task_reset"
    TOPIC_MAP = "/map"
    TOPIC_SIGNAL_MAP = "/signal_new_distance_map"

    SERVICE_DISTANCE_MAP = "/distance_map"
