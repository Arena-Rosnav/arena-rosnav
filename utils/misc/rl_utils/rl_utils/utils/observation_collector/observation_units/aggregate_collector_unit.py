from typing import Any, Dict

import numpy as np
import rospy
from costmap_2d.srv import GetDump
from pedsim_agents.utils import SemanticAttribute
from shared import Namespace

from .collector_unit import CollectorUnit

CloudPointDType = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("index", "<f4")]


class AggregateCollectorUnit(CollectorUnit):
    def __init__(
        self, ns: Namespace, observation_manager: "ObservationCollector"
    ) -> None:
        super().__init__(ns, observation_manager)
        self._get_dump_srv = rospy.ServiceProxy(
            self._ns("/move_base_flex/global_costmap/get_dump"), GetDump
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

        scans_msg = response.obstacle_layers[0].scans[0]

        semantic_layers = response.semantic_layers[0].layers
        semantic_info = {layer.type: layer.points for layer in semantic_layers}

        obs_dict.update(
            {
                "point_cloud": np.frombuffer(
                    scans_msg.data, dtype=np.dtype(CloudPointDType)
                ),
                **semantic_info,
            }
        )
        return obs_dict
