import dataclasses
import json

from task_generator.shared import rosparam_get
from task_generator.utils import Utils

import rospy
from task_generator.constants import Constants

from task_generator.tasks.modules import TM_Module
from task_generator.tasks.task_factory import TaskFactory

import map_generator.constants

import nav_msgs.srv as nav_srvs
import std_msgs.msg as std_msgs
import nav_msgs.msg as nav_msgs

from task_generator.manager.entity_manager.utils import ObstacleLayer
from task_generator.manager.utils import WorldMap

import dynamic_reconfigure.client

class MapInterface:
    TOPIC_REQUEST_MAP = "/request_new_map"
    TOPIC_MAP = "/map"

    SERVICE_MAP = "/static_map"

    __map_request_pub: rospy.Publisher
    __get_map_service: rospy.ServiceProxy
    

    def __init__(self, task):
        self._TASK = task

        # requests new map from map generator
        self.__map_request_pub = rospy.Publisher(
            self.TOPIC_REQUEST_MAP, std_msgs.String, queue_size=1
        )

        self.__get_map_service = rospy.ServiceProxy(
            self.SERVICE_MAP, nav_srvs.GetMap
        )

    def update_map(self):
        """
        Updates the map based on the distance information received from the service.
        """
        rospy.wait_for_message(self.TOPIC_MAP, nav_msgs.OccupancyGrid)
        dist_map: nav_srvs.GetMapResponse = self.__get_map_service()

        self._TASK.world_manager.update_world(
            world_map=WorldMap.from_distmap(distmap=dist_map)
        )
        self._TASK.obstacle_manager.reset(purge=ObstacleLayer.WORLD)
        self._TASK.obstacle_manager.spawn_world_obstacles(
            self._TASK.world_manager.world
        )

    def request_new_map(self, timeout):
        """
        Requests a new map from the map generator.
        """
        # set current eps immediately to 0 so that only one task
        # requests a new map

        triggered_by_me = not rosparam_get(bool, map_generator.constants.PARAM_GENERATING, False)

        if triggered_by_me:
            self.__map_request_pub.publish("")

        try:
            rospy.wait_for_message(
                self.TOPIC_MAP, nav_msgs.OccupancyGrid, timeout=timeout
            )
        except rospy.ROSException:
            rospy.logwarn(
                "[Map Generator] Timeout while waiting for new map. Continue with current map."
            )
        else:
            if triggered_by_me:
                rospy.loginfo("===================")
                rospy.loginfo("+++ Got new map +++")
                rospy.loginfo("===================")

@TaskFactory.register_module(Constants.TaskMode.TM_Module.MAP)
class TM_Map(TM_Module):

    @dataclasses.dataclass
    class Configuration:
        timeout: float = float('nan')
        algorithm: str = ''
        config_json: str = ''

    PARAM_ALGORITHM = map_generator.constants.MAP_GENERATOR_NS("algorithm")
    PARAM_ALGORITHM_CONFIG = map_generator.constants.MAP_GENERATOR_NS("algorithm_config")
    PARAM_TIMEOUT = map_generator.constants.MAP_GENERATOR_NS("generation_timeout")

    _map_interface: MapInterface
    _configuration: "Configuration"
    _dirty: bool

    def before_reset(self):
        if self._dirty:
            if Utils.get_arena_type() != Constants.ArenaType.TRAINING:
                self._map_interface.request_new_map(self._configuration.timeout)
                self._map_interface.update_map()
            self._dirty = False

    def reconfigure(self, config):

        if self._configuration.timeout != config["map_timeout"]:
            self._configuration.timeout = config["map_timeout"]
            self._dirty = True
        rospy.set_param(self.PARAM_TIMEOUT, self._configuration.timeout)

        if self._configuration.algorithm != config["map_algorithm"]:
            self._configuration.algorithm = config["map_algorithm"]
            self._dirty = True
        rospy.set_param(self.PARAM_ALGORITHM, self._configuration.algorithm)

        if self._configuration.config_json != config["map_config_json"]:
            self._configuration.config_json = config["map_config_json"]
            self._dirty = True
        config = json.loads(self._configuration.config_json)
        assert isinstance(config, dict)
        for k,v in config.items():
            rospy.set_param(self.PARAM_ALGORITHM_CONFIG(k), v)

    def __init__(self, **kwargs):
        """
        Initializes the Mod_DynamicMap module.
        """
        TM_Module.__init__(self, **kwargs)

        self._configuration = self.Configuration()

        self._map_interface = MapInterface(self._TASK)

        dynamic_reconfigure.client.Client(
            name=self.NODE_CONFIGURATION,
            config_callback=self.reconfigure
        )
        self._dirty = True