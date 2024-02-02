from typing import Dict
import genpy
import rospy
from task_generator.constants import Constants
from task_generator.shared import PositionOrientation
from task_generator.tasks import Task
from task_generator.tasks.modules import TM_Module
from task_generator.tasks.task_factory import TaskFactory

import geometry_msgs.msg as geometry_msgs

from tf.transformations import euler_from_quaternion

@TaskFactory.register_module(Constants.TaskMode.TM_Module.RVIZ_UI)
class Mod_OverrideRobot(TM_Module):
    TOPIC_SET_POSITION = "/initialpose"
    TOPIC_SET_GOAL = "/goalpose"
    TOPIC_NEW_SCENARIO = "/clicked_point"
    PARAM_WAYPOINTS = "guided_waypoints"


    _timeouts: Dict[int, genpy.Time]

    def __init__(self, task: Task, **kwargs):
        TM_Module.__init__(self, task, **kwargs)

        self._timeouts = dict()

        rospy.Subscriber(self.TOPIC_SET_POSITION,
                         geometry_msgs.PoseWithCovarianceStamped, self._cb_set_position)
        rospy.Subscriber(self.TOPIC_SET_GOAL,
                         geometry_msgs.PoseStamped, self._cb_set_goal)
        rospy.Subscriber(self.TOPIC_NEW_SCENARIO,
                         geometry_msgs.PointStamped, self._cb_new_scenario)
        
    def _reset_timeout(self, index: int):
        self._timeouts[index] = self._TASK.clock.clock

    def _cb_set_position(self, pos: geometry_msgs.PoseWithCovarianceStamped):
        poso = PositionOrientation(
            pos.pose.pose.position.x,
            pos.pose.pose.position.y,
            euler_from_quaternion(
                [
                    pos.pose.pose.orientation.x,
                    pos.pose.pose.orientation.y,
                    pos.pose.pose.orientation.z,
                    pos.pose.pose.orientation.w
                ]
            )[2]
        )

        self._TASK.set_robot_position(poso)

    def _cb_set_goal(self, pos: geometry_msgs.PoseStamped):
        poso = PositionOrientation(
            pos.pose.position.x,
            pos.pose.position.y,
            euler_from_quaternion(
                [
                    pos.pose.orientation.x,
                    pos.pose.orientation.y,
                    pos.pose.orientation.z,
                    pos.pose.orientation.w
                ]
            )[2]
        )

        self._TASK.set_robot_goal(poso)

    def _cb_new_scenario(self, *args, **kwargs):
        self._TASK.force_reset()