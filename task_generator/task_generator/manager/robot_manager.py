import dataclasses
from typing import Optional

import numpy as np
import rospy
import roslaunch
import os
import scipy.spatial.transform

import roslaunch
import rospy
from task_generator.constants import Constants, Config
from task_generator.manager.entity_manager.entity_manager import EntityManager
from task_generator.manager.entity_manager.utils import YAMLUtil
from task_generator.shared import ModelType, Namespace, PositionOrientation, Robot
from task_generator.utils import Utils, rosparam_get

from tf.transformations import quaternion_from_euler

import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as geometry_msgs
import std_srvs.srv as std_srvs


class RobotManager:
    """
    The robot manager manages the goal and start
    position of a robot for all task modes.
    """

    _namespace: Namespace

    _entity_manager: EntityManager

    _start_pos: PositionOrientation
    _goal_pos: PositionOrientation

    @property
    def start_pos(self) -> PositionOrientation:
        return self._start_pos

    @property
    def goal_pos(self) -> PositionOrientation:
        return self._goal_pos

    _position: PositionOrientation

    _robot_radius: float
    _goal_tolerance_distance: float
    _goal_tolerance_angle: float

    _robot: Robot

    _move_base_pub: rospy.Publisher
    _move_base_goal_pub: rospy.Publisher
    _pub_goal_timer: rospy.Timer
    _clear_costmaps_srv: rospy.ServiceProxy

    def __init__(
        self, namespace: Namespace, entity_manager: EntityManager, robot: Robot
    ):
        self._namespace = namespace

        self._entity_manager = entity_manager

        self._start_pos = PositionOrientation(0, 0, 0)
        self._goal_pos = PositionOrientation(0, 0, 0)

        self._goal_tolerance_distance = rosparam_get(
            float, "goal_radius", Config.Robot.GOAL_TOLERANCE_RADIUS
        )  # + self._robot_radius
        self._goal_tolerance_angle = rosparam_get(
            float, "goal_tolerance_angle", Config.Robot.GOAL_TOLERANCE_ANGLE
        )

        self._robot = robot
        self._safety_distance = rosparam_get(
            float,
            f"{robot.name}/safety_distance",
            Config.Robot.SPAWN_ROBOT_SAFE_DIST,
        )

        self._position = self._start_pos

    def set_up_robot(self):
        self._robot = dataclasses.replace(
            self._robot,
            model=self._robot.model.override(
                model_type=ModelType.YAML,
                override=lambda model: model.replace(
                    description=YAMLUtil.serialize(
                        YAMLUtil.update_plugins(
                            namespace=self.namespace,
                            description=YAMLUtil.parse_yaml(model.description),
                        )
                    )
                ),
            ),
        )

        self._entity_manager.spawn_robot(self._robot)

        # _gen_goal_topic = (
        #     self.namespace("goal")
        #     if Utils.get_arena_type() == Constants.ArenaType.TRAINING
        #     else self.namespace("move_base_simple", "goal")
        # )

        _gen_goal_topic = self.namespace("move_base_simple", "goal")

        self._move_base_goal_pub = rospy.Publisher(
            _gen_goal_topic, geometry_msgs.PoseStamped, queue_size=10
        )

        self._pub_goal_timer = rospy.Timer(
            rospy.Duration(nsecs=int(0.25e9)), self._publish_goal_periodically
        )

        rospy.Subscriber(
            self.namespace("odom"), nav_msgs.Odometry, self._robot_pos_callback
        )

        # if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
        #     return

        self._launch_robot()
        self._robot_radius = (
            float(rospy.get_param_cached("robot_radius"))
            if Utils.get_arena_type() == Constants.ArenaType.TRAINING
            else rosparam_get(float, self.namespace("robot_radius"))
        )

        # rospy.wait_for_service(os.path.join(self.namespace, "move_base", "clear_costmaps"))
        self._clear_costmaps_srv = rospy.ServiceProxy(
            self.namespace("move_base", "clear_costmaps"), std_srvs.Empty
        )

    @property
    def safe_distance(self) -> float:
        return self._robot_radius + self._safety_distance

    @property
    def model_name(self) -> str:
        return self._robot.model.name

    @property
    def name(self) -> str:
        return self._robot.name

    @property
    def namespace(self) -> Namespace:
        if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
            return Namespace(
                f"{self._namespace}{self._namespace}_{self.model_name}"
            )  # schizophrenia

        return self._namespace(self._robot.name)

    @property
    def is_done(self) -> bool:
        """kind of redundant right now, but could contain more logic in the future"""
        return self._is_goal_reached

    def move_robot_to_pos(self, position: PositionOrientation):
        self._entity_manager.move_robot(name=self.name, position=position)

    def reset(
        self,
        start_pos: Optional[PositionOrientation],
        goal_pos: Optional[PositionOrientation],
    ):
        """
        Publishes start and goal to data_recorder, publishes goal to move_base
        """

        if start_pos is not None:
            self._start_pos = start_pos
            self.move_robot_to_pos(start_pos)

            if self._robot.record_data_dir is not None:
                rospy.set_param(
                    self.namespace("start"), [float(v) for v in self._start_pos]
                )

        if goal_pos is not None:
            self._goal_pos = goal_pos
            self._publish_goal(self._goal_pos)

            if self._robot.record_data_dir is not None:
                rospy.set_param(
                    self.namespace("goal"), [float(v) for v in self._goal_pos]
                )

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
            np.array(goal[:2]) - np.array(start[:2])
        )

        # https://gamedev.stackexchange.com/a/4472
        angle_to_goal: float = np.pi - np.abs(np.abs(goal[2] - start[2]) - np.pi)

        return (
            distance_to_goal < self._goal_tolerance_distance
            and angle_to_goal < self._goal_tolerance_angle
        )

    def _publish_goal_periodically(self, *args, **kwargs):
        if self._goal_pos is not None:
            self._publish_goal(self._goal_pos)

    def _publish_goal(self, goal: PositionOrientation):
        goal_msg = geometry_msgs.PoseStamped()
        goal_msg.header.seq = 0
        goal_msg.header.stamp = rospy.get_rostime()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal.x
        goal_msg.pose.position.y = goal.y
        goal_msg.pose.position.z = 0

        goal_msg.pose.orientation = geometry_msgs.Quaternion(
            *quaternion_from_euler(0.0, 0.0, goal.orientation, axes="sxyz")
        )

        self._move_base_goal_pub.publish(goal_msg)

    def _launch_robot(self):
        rospy.logwarn(f"START WITH MODEL {self.namespace}")

        if Utils.get_arena_type() != Constants.ArenaType.TRAINING:
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(  # type: ignore
                ["arena_bringup", "robot.launch"]
            )

            args = [
                f"SIMULATOR:={Utils.get_simulator().value}",
                f"model:={self.model_name}",
                f"name:={self.name}",
                f"namespace:={self.namespace}",
                f"frame:={self.name+'/' if self.name != '' else ''}",
                f"inter_planner:={self._robot.inter_planner}",
                f"local_planner:={self._robot.local_planner}",
                f"complexity:={rosparam_get(int, 'complexity', 1)}",
                *(
                    [
                        "record_data:=true",
                        f"record_data_dir:={self._robot.record_data_dir}",
                    ]
                    if self._robot.record_data_dir is not None
                    else []
                ),
                f"train_mode:={rosparam_get(bool, 'train_mode', False)}",
                f"agent_name:={self._robot.agent}",
            ]

            self.process = roslaunch.parent.ROSLaunchParent(  # type: ignore
                roslaunch.rlutil.get_or_generate_uuid(None, False),  # type: ignore
                [(*roslaunch_file, args)],
            )
            self.process.start()

        # Overwrite default move base params
        base_frame: str = rospy.get_param_cached(self.namespace("robot_base_frame"))
        sensor_frame: str = rospy.get_param_cached(self.namespace("robot_sensor_frame"))

        rospy.set_param(
            self.namespace("move_base", "global_costmap", "robot_base_frame"),
            os.path.join(self.name, base_frame),
        )
        rospy.set_param(
            self.namespace("move_base", "local_costmap", "robot_base_frame"),
            os.path.join(self.name, base_frame),
        )
        rospy.set_param(
            self.namespace("move_base", "local_costmap", "scan", "sensor_frame"),
            os.path.join(self.name, sensor_frame),
        )
        rospy.set_param(
            self.namespace("move_base", "global_costmap", "scan", "sensor_frame"),
            os.path.join(self.name, base_frame),
        )

    def _robot_pos_callback(self, data: nav_msgs.Odometry):
        current_position = data.pose.pose
        quat = current_position.orientation

        rot = scipy.spatial.transform.Rotation.from_quat(
            [quat.x, quat.y, quat.z, quat.w]
        )

        self._position = PositionOrientation(
            current_position.position.x,
            current_position.position.y,
            rot.as_euler("xyz")[2],
        )
