from typing import Any, Dict

import numpy as np
import rospy
from nav_msgs.msg import Path
from task_generator.shared import Namespace

from ..constants import OBS_DICT_KEYS, TOPICS
from ..utils import pose3d_to_pose2d
from .collector_unit import CollectorUnit

from geometry_msgs.msg import PoseStamped


class GlobalplanCollectorUnit(CollectorUnit):
    """
    A class representing a GlobalplanCollectorUnit, which collects global plan observations.

    Attributes:
        _globalplan (np.ndarray): An array to store the global plan.
        _globalplan_sub (rospy.Subscriber): A subscriber for the global plan topic.
    """

    _globalplan: np.ndarray
    _globalplan_sub: rospy.Subscriber

    def __init__(
        self,
        ns: Namespace,
        observation_manager: "ObservationCollector",
        *args,
        **kwargs
    ) -> None:
        """
        Initializes a GlobalplanCollectorUnit instance.

        Args:
            ns (Namespace): The namespace for the collector unit.
            observation_manager (ObservationCollector): The observation manager for the collector unit.
        """
        super().__init__(ns, observation_manager)
        self._globalplan = np.array([])
        self._globalplan_sub: rospy.Subscriber = None
        self._subgoal = np.array([])
        self._subgoal_sub: rospy.Subscriber = None

    def init_subs(self):
        """
        Initializes the subscriber for the global plan topic.
        """
        self._globalplan_sub = rospy.Subscriber(
            self._ns(TOPICS.GLOBALPLAN), Path, self._cb_globalplan
        )
        self._subgoal_sub = rospy.Subscriber(
            self._ns(TOPICS.SUBGOAL), PoseStamped, self._cb_subgoal
        )

    def get_observations(
        self, obs_dict: Dict[str, Any], *args, **kwargs
    ) -> Dict[str, Any]:
        """
        Updates the observation dictionary with the global plan.

        Args:
            obs_dict (Dict[str, Any]): The observation dictionary.

        Returns:
            Dict[str, Any]: The updated observation dictionary.
        """
        obs_dict.update(
            {
                OBS_DICT_KEYS.GLOBAL_PLAN: self._globalplan,
                OBS_DICT_KEYS.SUBGOAL: self._subgoal,
            }
        )
        return obs_dict

    def _cb_globalplan(self, globalplan_msg: Path):
        """
        Callback function for processing the received global plan message.

        Args:
            globalplan_msg (Path): The received global plan message.
        """
        self._globalplan = GlobalplanCollectorUnit.process_global_plan_msg(
            globalplan_msg
        )

    def _cb_subgoal(self, subgoal_msg: PoseStamped):
        """
        Callback function for processing the received subgoal message.

        Args:
            subgoal_msg (PoseStamped): The received subgoal message.
        """
        self._subgoal = np.array(
            [subgoal_msg.pose.position.x, subgoal_msg.pose.position.y, 0.0]
        )

    @staticmethod
    def process_global_plan_msg(globalplan_msg: Path) -> np.ndarray:
        """
        Processes the received global plan message and converts it to a 2D numpy array.

        Args:
            globalplan_msg (Path): The received global plan message.

        Returns:
            np.ndarray: The processed global plan as a 2D numpy array.
        """
        global_plan_2d = list(
            map(
                lambda p: pose3d_to_pose2d(p.pose),
                globalplan_msg.poses,
            )
        )
        return np.array(list(map(lambda p2d: [p2d.x, p2d.y], global_plan_2d)))
