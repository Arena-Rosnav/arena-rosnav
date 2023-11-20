from typing import Any, Dict

import numpy as np
import rospy
from nav_msgs.msg import Path
from task_generator.shared import Namespace

from ..constants import OBS_DICT_KEYS, TOPICS
from ..utils import pose3d_to_pose2d
from .collector_unit import CollectorUnit


class GlobalplanCollectorUnit(CollectorUnit):
    _globalplan: np.ndarray

    def __init__(
        self, ns: Namespace, observation_manager: "ObservationCollector"
    ) -> None:
        super().__init__(ns, observation_manager)
        self._globalplan = np.array([])
        self._globalplan_sub: rospy.Subscriber = None

    def init_subs(self):
        self._globalplan_sub = rospy.Subscriber(
            self._ns(TOPICS.GLOBALPLAN), Path, self._cb_globalplan
        )

    def get_observations(
        self, obs_dict: Dict[str, Any], *args, **kwargs
    ) -> Dict[str, Any]:
        obs_dict.update({OBS_DICT_KEYS.GLOBAL_PLAN: self._globalplan})
        return obs_dict

    def _cb_globalplan(self, globalplan_msg: Path):
        self._globalplan = GlobalplanCollectorUnit.process_global_plan_msg(
            globalplan_msg
        )

    @staticmethod
    def process_global_plan_msg(globalplan_msg: Path) -> np.ndarray:
        global_plan_2d = list(
            map(
                lambda p: pose3d_to_pose2d(p.pose),
                globalplan_msg.poses,
            )
        )
        return np.array(list(map(lambda p2d: [p2d.x, p2d.y], global_plan_2d)))
