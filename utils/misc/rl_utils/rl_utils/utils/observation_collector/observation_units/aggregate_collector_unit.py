from typing import Any, Dict

import numpy as np
import rospy
from costmap_2d.srv import GetDump
from rl_utils.utils.rewards.utils import distances_from_pointcloud
from task_generator.shared import Namespace

from .collector_unit import CollectorUnit

CloudPointDType = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("index", "<f4")]


class AggregateCollectorUnit(CollectorUnit):
    def __init__(self, ns: Namespace, observation_manager: "ObservationCollector"):
        super().__init__(Namespace(ns), observation_manager)
        self._get_dump_srv = rospy.ServiceProxy(
            self._ns("move_base_flex/global_costmap/get_dump"), GetDump
        )

    def init_subs(self):
        pass

    def wait(self):
        pass

    def get_observations(
        self, obs_dict: Dict[str, Any], *args, **kwargs
    ) -> Dict[str, Any]:
        obs_dict = super().get_observations(obs_dict, *args, **kwargs)

        response = self._get_dump_srv.call()

        semantic_layers = response.semantic_layers[0].layers
        semantic_info = {layer.type: layer.points for layer in semantic_layers}

        point_cloud = AggregateCollectorUnit.cloudpoint_msg_to_numpy(
            response.obstacle_layers[0].scans[0]
        )
        obs_dict.update(
            {
                # "laser_scan": AggregateCollectorUnit.cloudpoint_to_laser_scan(
                #     point_cloud
                # ),
                "point_cloud": point_cloud,
                **semantic_info,
            }
        )
        return obs_dict

    @staticmethod
    def cloudpoint_to_laser_scan(cloud_point: np.ndarray) -> np.ndarray:
        return distances_from_pointcloud(cloud_point)

    @staticmethod
    def cloudpoint_msg_to_numpy(msg) -> np.ndarray:
        return np.frombuffer(msg.data, dtype=np.dtype(CloudPointDType))
