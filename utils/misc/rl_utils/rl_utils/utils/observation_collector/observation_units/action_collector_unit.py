from typing import Any, Dict
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from task_generator.shared import Namespace

from ..constants import OBS_DICT_KEYS, TOPICS
from .collector_unit import CollectorUnit


class ActionCollectorUnit(CollectorUnit):
    def __init__(
        self,
        ns: Namespace,
        observation_manager: "ObservationCollector",
        *args,
        **kwargs
    ) -> None:
        super().__init__(ns, observation_manager, *args, **kwargs)
        self._action = np.array([0, 0, 0])
        self._action_sub: rospy.Subscriber = None

    def init_subs(self):
        self._action_sub = rospy.Subscriber(
            self._ns(TOPICS.ACTION), Twist, self._cb_last_action
        )

    def get_observations(
        self, obs_dict: Dict[str, Any], *args, **kwargs
    ) -> Dict[str, Any]:
        obs_dict = super().get_observations(obs_dict, *args, **kwargs)

        if "last_action" in kwargs and (
            OBS_DICT_KEYS.CURR_ACTION not in obs_dict
            or obs_dict[OBS_DICT_KEYS.CURR_ACTION] is None
        ):
            obs_dict[OBS_DICT_KEYS.CURR_ACTION] = self._action
        else:
            obs_dict[OBS_DICT_KEYS.LAST_ACTION] = np.array(np.zeros_like(self._action))

        return obs_dict

    def _cb_last_action(self, action_msg: Twist):
        """
        Callback function for receiving last action data.

        Args:
            action_msg (Twist): Last action message.
        """
        self._action = np.array(
            [action_msg.linear.x, action_msg.linear.y, action_msg.angular.z]
        )
