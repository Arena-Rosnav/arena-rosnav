from typing import Dict
from builtin_interfaces.msg import Time

from task_generator.shared import PositionOrientation
from task_generator.tasks import Task
from task_generator.tasks.modules import TM_Module

import geometry_msgs.msg as geometry_msgs


class Mod_OverrideRobot(TM_Module):
    TOPIC_SET_POSITION = "/initialpose"
    TOPIC_SET_GOAL = "/goalpose"
    TOPIC_NEW_SCENARIO = "/clicked_point"
    PARAM_WAYPOINTS = "guided_waypoints"

    _timeouts: Dict[int, Time]

    def __init__(self, task: Task, **kwargs):
        TM_Module.__init__(self, task, **kwargs)

        self._timeouts = {}

        self.node.create_subscription(
            geometry_msgs.PoseWithCovarianceStamped,
            self.TOPIC_SET_POSITION,
            self._cb_set_position,
            1
        )

        self.node.create_subscription(
            geometry_msgs.PoseWithCovarianceStamped,
            self.TOPIC_SET_GOAL,
            self._cb_set_goal,
            1
        )

        self.node.create_subscription(
            geometry_msgs.PointStamped,
            self.TOPIC_NEW_SCENARIO,
            self._cb_new_scenario,
            1
        )

    def _reset_timeout(self, index: int):
        self._timeouts[index] = self._TASK.clock.clock

    def _cb_set_position(self, pos: geometry_msgs.PoseWithCovarianceStamped):
        self._TASK.set_robot_position(
            PositionOrientation.from_pose(
                pos.pose.pose
            )
        )

    def _cb_set_goal(self, pos: geometry_msgs.PoseStamped):
        self._TASK.set_robot_goal(
            PositionOrientation.from_pose(
                pos.pose
            )
        )

    def _cb_new_scenario(self, *args, **kwargs):
        self._TASK.force_reset()
