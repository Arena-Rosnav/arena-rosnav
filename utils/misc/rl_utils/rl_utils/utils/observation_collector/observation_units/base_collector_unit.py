from time import sleep
from typing import Any, Dict

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Pose2D, PoseStamped
from nav_msgs.msg import Odometry
from rl_utils.utils.observation_collector.observation_units.semantic_ped_unit import (
    SemanticAggregateUnit,
)
from sensor_msgs.msg import LaserScan
from task_generator.shared import Namespace

from ..constants import OBS_DICT_KEYS, TOPICS
from ..utils import get_goal_pose_in_robot_frame, pose3d_to_pose2d
from .collector_unit import CollectorUnit


class BaseCollectorUnit(CollectorUnit):
    """
    A class for collecting basic navigation information such as robot state, sensor data, goal.

    Attributes:
        _robot_state (Odometry): Current robot state.
        _robot_pose (Pose2D): Current robot pose.
        _laser (np.ndarray): Current laser scan data.
        _full_range_laser (np.ndarray): Current full range laser scan data. (For detecting collision in blind spot)
        _goal (Pose2D): Episode's goal pose.

        _scan_sub (rospy.Subscriber): Subscriber for laser scan data.
        _full_scan_sub (rospy.Subscriber): Subscriber for full range laser scan data.
        _robot_state_sub (rospy.Subscriber): Subscriber for robot state data.
        _goal_sub (rospy.Subscriber): Subscriber for goal data.

        _received_odom (bool): Flag indicating if robot state data has been received.
        _received_scan (bool): Flag indicating if laser scan data has been received.
        _received_goal (bool): Flag indicating if goal data has been received.

        _first_reset (bool): Flag indicating if it is the first reset.
    """

    # Retrieved information
    _robot_state: Odometry
    _robot_pose: Pose2D
    _laser: np.ndarray
    _full_range_laser: np.ndarray
    _goal: Pose2D

    # Subscriptions
    _scan_sub: rospy.Subscriber
    _full_scan_sub: rospy.Subscriber
    _robot_state_sub: rospy.Subscriber
    _goal_sub: rospy.Subscriber

    # Received Flags
    _received_odom: bool
    _received_scan: bool
    _received_goal: bool

    _first_reset: bool

    def __init__(
        self,
        ns: Namespace,
        observation_manager,
        subgoal_mode: bool = False,
        *args,
        **kwargs
    ) -> None:
        """
        Initialize the BaseCollectorUnit.

        Args:
            ns (Namespace): Namespace for the collector unit.
            observation_manager: Observation manager holding this collector unit.
        """
        super().__init__(ns, observation_manager)
        self._laser_num_beams = rospy.get_param("laser/num_beams")
        self._enable_full_range_laser = rospy.get_param("laser/full_range_laser", False)

        self._robot_state = Odometry()
        self._robot_pose = Pose2D()
        self._laser = np.array([])
        self._full_range_laser = np.array([])
        self._goal = Pose2D()
        self._goal_msg = PoseStamped()

        self._scan_sub: rospy.Subscriber = None
        self._full_scan_sub: rospy.Subscriber = None
        self._robot_state_sub: rospy.Subscriber = None
        self._goal_sub: rospy.Subscriber = None

        self._received_odom = False
        self._received_scan = False
        self._received_goal = False

        self._subgoal_mode = subgoal_mode

        self._first_reset = True

    def init_subs(self):
        """
        Initialize the subscribers for robot state and sensor data.
        """
        self._scan_sub = rospy.Subscriber(
            self._ns(TOPICS.LASER),
            LaserScan,
            self._cb_laser,
            tcp_nodelay=True,
        )
        if self._enable_full_range_laser:
            self._full_scan_sub = rospy.Subscriber(
                self._ns(TOPICS.FULL_RANGE_LASER),
                LaserScan,
                self._cb_full_range_laser,
                tcp_nodelay=True,
            )
        self._robot_state_sub = rospy.Subscriber(
            self._ns(TOPICS.ROBOT_STATE),
            Odometry,
            self._cb_robot_state,
            tcp_nodelay=True,
        )
        self._goal_sub = rospy.Subscriber(
            (
                self._ns(TOPICS.GOAL)
                if not self._subgoal_mode
                else self._ns(TOPICS.SUBGOAL)
            ),
            PoseStamped,
            self._cb_goal,
            tcp_nodelay=True,
        )

    def wait(self):
        """
        Wait for the required data to be received.
        """
        pass
        # if self._first_reset:
        #     self._first_reset = False
        #     return

        # for _ in range(int(MAX_WAIT / SLEEP)):
        #     if self._received_odom and self._received_scan and self._received_goal:
        #         return

        #     sleep(SLEEP)

        # raise TimeoutError(
        #     f"Couldn't retrieve data for: {false_params(odom=self._received_odom, laser=self._received_scan, goal=self._received_goal)}"
        # )

    def get_observations(
        self, obs_dict: Dict[str, Any], *args, **kwargs
    ) -> Dict[str, Any]:
        """
        Get the observations from the collected data.

        Args:
            obs_dict (Dict[str, Any]): Dictionary to store the observations.

        Returns:
            Dict[str, Any]: Updated dictionary with observations.
        """
        obs_dict = super().get_observations(obs_dict)

        goal_in_robot_frame = SemanticAggregateUnit.get_relative_pos_to_robot(
            self._robot_pose,
            np.array([[self._goal.x, self._goal.y, 1]]),
        ).squeeze(0)

        dist_to_goal = np.linalg.norm(goal_in_robot_frame)
        angle_to_goal = np.arctan2(goal_in_robot_frame[1], goal_in_robot_frame[0])

        obs_dict.update(
            {
                OBS_DICT_KEYS.LASER: self._laser,
                OBS_DICT_KEYS.ROBOT_POSE: self._robot_pose,
                OBS_DICT_KEYS.GOAL: self._goal_msg,
                OBS_DICT_KEYS.GOAL_DIST_ANGLE: (
                    dist_to_goal,
                    angle_to_goal,
                ),
                OBS_DICT_KEYS.GOAL_LOCATION: (self._goal.x, self._goal.y),
                OBS_DICT_KEYS.GOAL_LOCATION_IN_ROBOT_FRAME: goal_in_robot_frame,
                OBS_DICT_KEYS.DISTANCE_TO_GOAL: dist_to_goal,
                OBS_DICT_KEYS.LAST_ACTION: kwargs.get(
                    "last_action", np.array([0, 0, 0])
                ),
            }
        )

        if self._enable_full_range_laser:
            obs_dict.update({"full_laser_scan": self._full_range_laser})

        return obs_dict

    def _cb_laser(self, laser_msg: LaserScan):
        """
        Callback function for receiving laser scan data.

        Args:
            laser_msg (LaserScan): Laser scan message.
        """
        self._received_scan = True
        self._laser = BaseCollectorUnit.process_laser_msg(
            laser_msg=laser_msg, laser_num_beams=self._laser_num_beams
        )

    def _cb_full_range_laser(self, laser_msg: LaserScan):
        """
        Callback function for receiving full range laser scan data.

        Args:
            laser_msg (LaserScan): Full range laser scan message.
        """
        self._full_range_laser = BaseCollectorUnit.process_laser_msg(
            laser_msg=laser_msg,
            laser_num_beams=self._laser_num_beams,
        )

    def _cb_robot_state(self, robot_state_msg: Odometry):
        """
        Callback function for receiving robot state data.

        Args:
            robot_state_msg (Odometry): Robot state message.
        """
        self._received_odom = True
        self._robot_state = robot_state_msg
        self._robot_pose = pose3d_to_pose2d(self._robot_state.pose.pose)

    def _cb_goal(self, goal_msg: PoseStamped):
        """
        Callback function for receiving goal data.

        Args:
            goal_msg (PoseStamped): Goal message.
        """
        self._received_goal = True
        self._goal_msg = goal_msg
        self._goal = pose3d_to_pose2d(goal_msg.pose)

    @staticmethod
    def process_laser_msg(laser_msg: LaserScan, laser_num_beams: int) -> np.ndarray:
        """
        Process the received laser scan message.

        Args:
            laser_msg (LaserScan): Laser scan message.
            laser_num_beams (int): Number of laser beams.

        Returns:
            np.ndarray: Processed laser scan data.
        """
        if len(laser_msg.ranges) == 0:
            return np.zeros(laser_num_beams, dtype=float)

        laser = np.array(laser_msg.ranges, np.float32)
        laser[np.isnan(laser)] = laser_msg.range_max
        return laser

    @staticmethod
    def process_robot_state_msg(pose: Pose) -> Pose2D:
        """
        Process the received robot state message.

        Args:
            pose (Pose): Robot pose message.

        Returns:
            Pose2D: Processed robot pose data.
        """
        return pose3d_to_pose2d(pose)
