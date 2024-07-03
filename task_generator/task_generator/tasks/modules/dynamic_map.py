import os
from typing import Dict
import map_distance_server.srv as map_distance_server_srvs
import std_msgs.msg as std_msgs
import nav_msgs.msg as nav_msgs
from task_generator.constants import Constants

from task_generator.manager.entity_manager.utils import ObstacleLayer
from task_generator.manager.utils import WorldMap

import rospy
from task_generator.tasks.modules import TM_Module
from task_generator.tasks.task_factory import TaskFactory
from task_generator.utils import rosparam_get

from map_generator.constants import MAP_GENERATOR_NS

# DYNAMIC MAP INTERFACE

DynamicMapConfiguration = Dict[str, Dict]


@TaskFactory.register_module(Constants.TaskMode.TM_Module.DYNAMIC_MAP)
class Mod_DynamicMap(TM_Module):
    """
    This class represents a module for generating and managing dynamic maps in the task generator.
    It provides functionality for requesting new maps, resetting tasks, and updating the map based on distance information.
    """

    __map_request_pub: rospy.Publisher
    __task_reset_pub: rospy.Publisher
    __get_dist_map_service: rospy.ServiceProxy

    PARAM_MAP_FILE = "map_file"
    PARAM_EPISODES = "/dynamic_map/curr_eps"
    PARAM_GENERATOR = "generator"
    PARAM_GENERATOR_CONFIGS = "/generator_configs"

    TOPIC_REQUEST_MAP = "/request_new_map"
    TOPIC_RESET = "/dynamic_map/task_reset"
    TOPIC_MAP = "/map"
    TOPIC_SIGNAL_MAP = "/signal_new_distance_map"

    SERVICE_DISTANCE_MAP = "/distance_map"

    def before_reset(self, **kwargs):
        """
        This method is called before resetting the task.
        It increments the number of episodes and requests a new map if the episode count exceeds the threshold.
        """
        self._episodes += 1

        if self._episodes >= self._target_eps_num:
            self.request_new_map()

    def __init__(self, **kwargs):
        """
        Initializes the Mod_DynamicMap module.
        """
        TM_Module.__init__(self, **kwargs)

        rospy.Subscriber(self.TOPIC_RESET, std_msgs.String, self._cb_task_reset)

        # requests new map from map generator
        self.__map_request_pub = rospy.Publisher(
            self.TOPIC_REQUEST_MAP, std_msgs.String, queue_size=1
        )
        # task reset for all taskmanagers when one resets
        self.__task_reset_pub = rospy.Publisher(
            self.TOPIC_RESET, std_msgs.String, queue_size=1
        )

        self.__get_dist_map_service = rospy.ServiceProxy(
            self.SERVICE_DISTANCE_MAP, map_distance_server_srvs.GetDistanceMap
        )

        num_envs: int = (
            rosparam_get(int, "num_envs", 1)
            if "eval_sim" not in self._TASK.robot_managers[0].namespace
            else 1
        )
        self._target_eps_num = (
            rosparam_get(int, MAP_GENERATOR_NS("episode_per_map"), 1) * num_envs
        )

    def _set_config(self, config: DynamicMapConfiguration):
        """
        Sets the configuration for the map generator based on the provided DynamicMapConfiguration object.
        """
        generator = rosparam_get(str, self.PARAM_GENERATOR)

        log = f"Setting [Map Generator: {generator}] parameters"

        config_generator = config.get(generator, dict())

        for key, value in config_generator.items():
            log += f"\t{key}={value}"
            rospy.set_param(
                os.path.join(self.PARAM_GENERATOR_CONFIGS, generator, key), value
            )

        rospy.loginfo(log)

    def _cb_task_reset(self, *args, **kwargs):
        """
        Callback function for task reset.
        Updates the map manager and triggers map update.
        """
        # task reset for all taskmanagers when one resets
        # update map manager

        self._update_map()

    def _update_map(self):
        """
        Updates the map based on the distance information received from the service.
        """
        dist_map: map_distance_server_srvs.GetDistanceMapResponse = (
            self.__get_dist_map_service()
        )

        if isinstance(dist_map, map_distance_server_srvs.GetDistanceMapResponse):
            self._TASK.world_manager.update_world(
                world_map=WorldMap.from_distmap(distmap=dist_map)
            )
            self._TASK.obstacle_manager.reset(purge=ObstacleLayer.WORLD)
            self._TASK.obstacle_manager.spawn_world_obstacles(
                self._TASK.world_manager.world
            )

    def request_new_map(self):
        """
        Requests a new map from the map generator.
        """
        # set current eps immediately to 0 so that only one task
        # requests a new map
        self._episodes = 0

        self.__map_request_pub.publish("")

        try:
            rospy.wait_for_message(self.TOPIC_MAP, nav_msgs.OccupancyGrid, timeout=60)
            rospy.wait_for_message(self.TOPIC_SIGNAL_MAP, std_msgs.String, timeout=120)
        except rospy.ROSException:
            rospy.logwarn(
                "[Map Generator] Timeout while waiting for new map. Continue with current map."
            )
        else:
            rospy.loginfo("===================")
            rospy.loginfo("+++ Got new map +++")
            rospy.loginfo("===================")

        self.__task_reset_pub.publish("")

    @property
    def _episodes(self) -> float:
        """
        Property representing the current number of episodes.
        """
        try:
            return rosparam_get(float, self.PARAM_EPISODES, float("inf"))
        except Exception as e:
            rospy.logwarn(e)
            return 0

    @_episodes.setter
    def _episodes(self, value: float):
        """
        Setter for the _episodes property.
        """
        try:
            rospy.set_param(self.PARAM_EPISODES, value)
        except Exception as e:
            rospy.logwarn(e)
