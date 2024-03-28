import functools
from typing import Any, Dict, Generic, List, Optional, TypeVar

import numpy as np
import rospy
from rl_utils.utils.rewards.utils import distances_from_pointcloud
from task_generator.shared import Namespace

import sensor_msgs.msg as sensor_msgs
import crowdsim_msgs.msg as crowdsim_msgs
from crowdsim_agents.utils import SemanticAttribute

from rl_utils.utils.observation_collector.observation_units.collector_unit import (
    CollectorUnit,
)
import rl_utils.utils.observation_collector.constants as Constants

CloudPointDType = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("index", "<f4")]

T = TypeVar("T")


class AggregateCollectorUnit(CollectorUnit):
    """
    A class that represents an aggregate collector unit for observations.

    This class collects and manages different types of observations, including laser observations and semantic observations.

    Attributes:
        _laser_observations (List[LaserObservation]): A list of laser observations.
        _semantic_observations (Dict[Constants.OBS_DICT_KEYS.SEMANTIC, SemanticObservation]): A dictionary of semantic observations.

    Methods:
        __init__(self, ns: Namespace, observation_manager: "ObservationCollector"): Initializes the AggregateCollectorUnit.
        init_subs(self): Initializes the subscribers for the observations.
        get_observations(self, obs_dict: Dict[str, Any], *args, **kwargs) -> Dict[str, Any]: Retrieves the observations.
        wait(self): Waits for any pending operations to complete.
        cloudpoint_to_laser_scan(cloud_point: np.ndarray) -> np.ndarray: Converts a cloud point to a laser scan.
        cloudpoint_msg_to_numpy(msg) -> np.ndarray: Converts a cloud point message to a numpy array.
    """

    class Staleable:
        _stale: bool

        def __init__(self):
            self._stale = True

        @property
        def stale(self) -> bool:
            return self._stale

        def invalidate(self):
            self._stale = True

    class Observation(Staleable, Generic[T]):
        _value: T

        def __init__(self, initial_value: T) -> None:
            super().__init__()
            self._value = initial_value

        @property
        def value(self) -> T:
            return self._value

        def update(self, value: T):
            self._value = value
            self._stale = False

    LaserObservation = Observation[sensor_msgs.LaserScan]
    SemanticObservation = Observation[crowdsim_msgs.SemanticData]

    _laser_observations: List[LaserObservation]
    _semantic_observations: Dict[Constants.OBS_DICT_KEYS.SEMANTIC, SemanticObservation]

    def __init__(
        self,
        ns: Namespace,
        observation_manager: "ObservationCollector",
        *args,
        **kwargs
    ):
        super().__init__(Namespace(ns), observation_manager)

        self._laser_observations = []
        self._semantic_observations = {
            semantic_type.value: self.SemanticObservation(crowdsim_msgs.SemanticData())
            for semantic_type in SemanticAttribute
        }

    def init_subs(self):
        local_costmap_conf = rospy.get_param(
            self._ns("move_base_flex", "global_costmap"), {}
        )

        def obstacle_layer(config: dict):
            observation = config.get("observation_sources", None)
            observation_config = config.get(observation, {})
            if observation_config.get("data_type", "") == "LaserScan":
                observation_container = self.LaserObservation(sensor_msgs.LaserScan())
                self._laser_observations.append(observation_container)

                rospy.Subscriber(
                    observation_config.get("topic"),
                    sensor_msgs.LaserScan,
                    functools.partial(observation_container.update),
                )

        def semantic_layer(config: dict):
            for observation in config.get("observation_sources", "").split(" "):
                try:
                    semantic_attribute = Constants.OBS_DICT_KEYS.SEMANTIC(observation)
                except ValueError:
                    pass
                else:
                    observation_config = config.get(observation, {})

                    observation_container = self.SemanticObservation(
                        crowdsim_msgs.SemanticData()
                    )
                    self._semantic_observations[semantic_attribute.value] = (
                        observation_container
                    )

                    rospy.Subscriber(
                        self._ns.simulation_ns + observation_config.get("topic"),
                        crowdsim_msgs.SemanticData,
                        functools.partial(observation_container.update),
                    )

        for plugin in local_costmap_conf.get("plugins", []):
            plugin_name = plugin.get("name")
            plugin_type = plugin.get("type")

            if plugin_type == "costmap_2d::ObstacleLayer":
                obstacle_layer(local_costmap_conf.get(plugin_name))
            elif plugin_type == "costmap_2d::SemanticLayer":
                semantic_layer(local_costmap_conf.get(plugin_name))
            else:
                pass

    def get_observations(
        self, obs_dict: Dict[str, Any], *args, **kwargs
    ) -> Dict[str, Any]:
        # if any(not observation.stale for observation in self._laser_observations):
        #     # very basic, won't work with most multi-sensor setups
        #     obs_dict[Constants.OBS_DICT_KEYS.LASER] = np.minimum.accumulate(
        #         [obvervation.value.ranges for obvervation in self._laser_observations]
        #     )

        #     for obs in self._laser_observations:
        #         obs.invalidate()

        for semantic_attribute, observation in self._semantic_observations.items():
            obs_dict[semantic_attribute] = observation.value

        return obs_dict

    def wait(self):
        pass
