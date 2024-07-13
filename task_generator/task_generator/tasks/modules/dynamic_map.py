import json
import os
from typing import Dict

from task_generator.constants import Constants

import std_msgs.msg as std_msgs

import rospy
from task_generator.tasks.modules import TM_Module
from task_generator.tasks.task_factory import TaskFactory
from task_generator.utils import rosparam_get
from task_generator.tasks.modules.map import MapInterface

from map_generator.constants import MAP_GENERATOR_NS

import dynamic_reconfigure.client

# DYNAMIC MAP INTERFACE

DynamicMapConfiguration = Dict[str, Dict]


@TaskFactory.register_module(Constants.TaskMode.TM_Module.DYNAMIC_MAP)
class Mod_DynamicMap(TM_Module):
    """
    This class represents a module for generating and managing dynamic maps in the task generator.
    It provides functionality for requesting new maps, resetting tasks, and updating the map based on distance information.
    """

    PARAM_EPISODES_PER_MAP = MAP_GENERATOR_NS("episode_per_map")
    PARAM_EPISODES = "/dynamic_map/curr_eps"
    PARAM_GENERATOR = "generator"
    PARAM_GENERATOR_CONFIGS = "/generator_configs"

    TOPIC_RESET = "/dynamic_map/task_reset"

    _map_interface: MapInterface
    __task_reset_pub: rospy.Publisher

    def before_reset(self, **kwargs):
        """
        This method is called before resetting the task.
        It increments the number of episodes and requests a new map if the episode count exceeds the threshold.
        """

        self._target_eps_num = (
            rosparam_get(int, MAP_GENERATOR_NS("episode_per_map"), 1) * self._num_envs
        )
        self._generation_timeout = rosparam_get(
            float, MAP_GENERATOR_NS("generation_timeout"), 60
        )

        self._episodes += 1

        if self._episodes >= self._target_eps_num:
            self.request_new_map()
            self._map_interface.update_map()

    def __init__(self, **kwargs):
        """
        Initializes the Mod_DynamicMap module.
        """
        TM_Module.__init__(self, **kwargs)

        # task reset for all taskmanagers when one resets
        self.__task_reset_pub = rospy.Publisher(
            self.TOPIC_RESET, std_msgs.String, queue_size=1
        )

        self._num_envs: int = (
            rosparam_get(int, "num_envs", 1)
            if "eval_sim" not in self._TASK.robot_managers[0].namespace
            else 1
        )

        dynamic_reconfigure.client.Client(
            name=self.NODE_CONFIGURATION,
            config_callback=self.reconfigure
        )

        self._map_interface = MapInterface(self._TASK)

    def reconfigure(self, config):
        rospy.set_param(self.PARAM_EPISODES_PER_MAP, config["DYNAMICMAP_episodes"])

    def _set_config(self, config: DynamicMapConfiguration):
        """
        Sets the configuration for the map generator based on the provided DynamicMapConfiguration object.
        """
        generator = rosparam_get(str, MAP_GENERATOR_NS(self.PARAM_GENERATOR))

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

        self._map_interface.update_map()


    def request_new_map(self):
        self._episodes = 0
        self._map_interface.request_new_map(self._generation_timeout)
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
