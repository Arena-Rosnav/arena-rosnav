from typing import Callable

import numpy as np
import rospy
import roslaunch
import os
import time

from nav_msgs.msg import Odometry
import os
import time

import roslaunch
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from task_generator.constants import Constants
from task_generator.shared import Position, Robot
from task_generator.simulators.base_simulator import BaseSimulator
from task_generator.utils import Utils


class RobotManager:
    """
    The robot manager manages the goal and start
    position of a robot for all task modes.
    """

    _ns_prefix: Callable[..., str]

    _simulator: BaseSimulator

    _start_pos: Position
    _goal_pos: Position
    _position: Position

    _robot_radius: float
    _goal_radius: float

    _robot: Robot

    _move_base_pub: rospy.Publisher
    _move_base_goal_pub: rospy.Publisher
    _pub_goal_timer: rospy.Timer
    _clear_costmaps_srv: rospy.ServiceProxy

    def __init__(self, simulator: BaseSimulator, robot: Robot):
        self._ns_prefix = lambda *topic: os.path.join(
            self._robot.namespace, *topic)

        self._simulator = simulator

        self._start_pos = (0, 0)
        self._goal_pos = (0, 0)

        self._goal_radius = float(str(rospy.get_param("goal_radius", 0.7))) + 1

        self._robot = robot
        # and rospy.get_param('task_mode', 'scenario') == 'scenario'

        self._position = self._start_pos

    def set_up_robot(self):
        if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
            self._robot_radius = float(str(rospy.get_param("robot_radius")))

        self._simulator.spawn_robot(self._robot)

        self._move_base_goal_pub = rospy.Publisher(self._ns_prefix(
            "move_base_simple", "goal"), PoseStamped, queue_size=10)
        self._pub_goal_timer = rospy.Timer(rospy.Duration(
            nsecs=int(0.25e9)), self._publish_goal_periodically)

        rospy.Subscriber(
            self._ns_prefix("odom"),
            Odometry,
            self._robot_pos_callback
        )

        if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
            return

        self._launch_robot()

        self._robot_radius = float(
            str(rospy.get_param(self._ns_prefix("robot_radius"))))

        # rospy.wait_for_service(os.path.join(self.namespace, "move_base", "clear_costmaps"))
        self._clear_costmaps_srv = rospy.ServiceProxy(
            self._ns_prefix("move_base", "clear_costmaps"),
            Empty
        )

    @property
    def safe_distance(self) -> float:
        return self._robot_radius + Constants.RobotManager.SPAWN_ROBOT_SAFE_DIST

    @property
    def name(self) -> str:
        if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
            return ""

        return self._robot.namespace

    @property
    def namespace(self) -> str:
        return self._robot.namespace

    @property
    def is_done(self) -> bool:
        """kind of redundant right now, but could contain more logic in the future"""
        return self._is_goal_reached

    def move_robot_to_pos(self, pos: Position):
        self._simulator.move_entity(name=self._robot.namespace, pos=(*pos, 0))

    def reset(self, start_pos: Position, goal_pos: Position):
        """
            Publishes start and goal to data_recorder, publishes goal to move_base
        """
        self._start_pos, self._goal_pos = start_pos, goal_pos

        if self._robot.record_data:
            rospy.set_param(os.path.join(self._robot.namespace, "goal"),
                            str(list(self._goal_pos)))
            rospy.set_param(os.path.join(self._robot.namespace,
                            "start"), str(list(self._start_pos)))

        self._publish_goal(self._goal_pos)
        self.move_robot_to_pos(self._start_pos)

        time.sleep(0.1)

        try:
            self._clear_costmaps_srv()
        except:
            pass

        return self._position, self._goal_pos

    @property
    def _is_goal_reached(self) -> bool:

        start = self._position
        goal = self._goal_pos

        distance_to_goal: float = np.linalg.norm(
            np.array(goal) - np.array(start))

        return distance_to_goal < self._goal_radius

    def _publish_goal_periodically(self, *args, **kwargs):
        if self._goal_pos is not None:
            self._publish_goal(self._goal_pos)

    def _publish_goal(self, goal: Position):
        goal_msg = PoseStamped()
        goal_msg.header.seq = 0
        goal_msg.header.stamp = rospy.get_rostime()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal[0]
        goal_msg.pose.position.y = goal[1]

        goal_msg.pose.orientation.w = 0
        goal_msg.pose.orientation.x = 0
        goal_msg.pose.orientation.y = 0
        goal_msg.pose.orientation.z = 1

        self._move_base_goal_pub.publish(goal_msg)

    def _launch_robot(self):
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(  # type: ignore
            ["arena_bringup", "robot.launch"]
        )

        rospy.loginfo(f"START WITH MODEL {self._robot.namespace}")

        args = [
            f"model:={self._robot.model.get().name}",
            f"local_planner:={self._robot.planner}",
            f"namespace:={self._robot.namespace}",
            f"complexity:={rospy.get_param('complexity', 1)}",
            f"record_data:={self._robot.record_data}",
            f"agent_name:={self._robot.agent}"
        ]

        self.process = roslaunch.parent.ROSLaunchParent(  # type: ignore
            roslaunch.rlutil.get_or_generate_uuid(None, False),  # type: ignore
            [(*roslaunch_file, args)]
        )
        self.process.start()

        # Overwrite default move base params
        base_frame: str = str(rospy.get_param(
            os.path.join(self._robot.namespace, "robot_base_frame")))
        sensor_frame: str = str(rospy.get_param(
            os.path.join(self._robot.namespace, "robot_sensor_frame")))

        rospy.set_param(
            os.path.join(self._robot.namespace, "move_base",
                         "global_costmap", "robot_base_frame"),
            self._robot.namespace.replace("/", "") + "/" + base_frame
        )
        rospy.set_param(
            os.path.join(self._robot.namespace, "move_base",
                         "local_costmap", "robot_base_frame"),
            self._robot.namespace.replace("/", "") + "/" + base_frame
        )
        rospy.set_param(
            os.path.join(self._robot.namespace, "move_base",
                         "local_costmap", "scan", "sensor_frame"),
            self._robot.namespace.replace("/", "") + "/" + sensor_frame
        )
        rospy.set_param(
            os.path.join(self._robot.namespace, "move_base",
                         "global_costmap", "scan", "sensor_frame"),
            self._robot.namespace.replace("/", "") + "/" + base_frame
        )

    def _robot_pos_callback(self, data: Odometry):
        current_position = data.pose.pose.position

        self._position = (current_position.x, current_position.y)
