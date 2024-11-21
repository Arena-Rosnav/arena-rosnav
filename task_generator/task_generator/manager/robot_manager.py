import dataclasses
import os
import typing

import numpy as np
import scipy.spatial.transform
import rclpy

from task_generator import NodeInterface
import task_generator.utils.arena as Utils
from task_generator.utils.geometry import quaternion_from_euler
from task_generator.constants import Constants
from task_generator.manager.entity_manager import EntityManager
from task_generator.manager.entity_manager.utils import YAMLUtil
from task_generator.shared import ModelType, Namespace, PositionOrientation, Robot

import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as geometry_msgs
import std_srvs.srv as std_srvs

import launch
import ament_index_python


class RobotManager(NodeInterface):
    """
    The robot manager manages the goal and start
    position of a robot for all task modes.
    """

    _namespace: Namespace
    _entity_manager: EntityManager
    _start_pos: PositionOrientation
    _goal_pos: PositionOrientation
    _position: PositionOrientation
    _robot_radius: float
    _goal_tolerance_distance: float
    _goal_tolerance_angle: float
    _robot: Robot
    _move_base_pub: rclpy.publisher.Publisher
    _move_base_goal_pub: rclpy.publisher.Publisher
    _pub_goal_timer: rclpy.timer.Timer
    _clear_costmaps_srv: rclpy.client.Client

    @property
    def start_pos(self) -> PositionOrientation:
        return self._start_pos

    @property
    def goal_pos(self) -> PositionOrientation:
        return self._goal_pos

    def __init__(
        self, namespace: Namespace, entity_manager: EntityManager, robot: Robot
    ):
        NodeInterface.__init__(self)

        self._namespace = namespace
        self._entity_manager = entity_manager
        self._start_pos = PositionOrientation(0, 0, 0)
        self._goal_pos = PositionOrientation(0, 0, 0)

        # Parameter handling
        try:
            self._goal_tolerance_distance = self.node.Configuration.Robot.GOAL_TOLERANCE_RADIUS.value
            self._goal_tolerance_angle = self.node.Configuration.Robot.GOAL_TOLERANCE_ANGLE.value
            self._safety_distance = self.node.Configuration.Robot.SPAWN_ROBOT_SAFE_DIST.value
        except Exception as e:
            # Fallback values
            self._goal_tolerance_distance = 1.0
            self._goal_tolerance_angle = 0.523599
            self._safety_distance = 0.25
            print(f"Warning: Using default values for robot parameters: {e}")

        self._robot = robot
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

        _gen_goal_topic = self.namespace("move_base_simple", "goal")

        self._move_base_goal_pub = self.node.create_publisher(
            geometry_msgs.PoseStamped, _gen_goal_topic, 10
        )

        self._pub_goal_timer = self.node.create_timer(
            0.25, self._publish_goal_periodically
        )
        self.node.create_subscription(
            nav_msgs.Odometry, self.namespace(
                "odom"), self._robot_pos_callback, 10
        )

        self._launch_robot()

        self._robot_radius = self.node.rosparam_get('robot_radius', 0.25)

        self._clear_costmaps_srv = self.node.create_client(
            std_srvs.Empty, self.namespace("move_base", "clear_costmaps"))

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
            )

        return self._namespace(self._robot.name)

    @property
    def is_done(self) -> bool:
        return self._is_goal_reached

    def move_robot_to_pos(self, position: PositionOrientation):
        self._entity_manager.move_robot(name=self.name, position=position)

    def reset(
        self,
        start_pos: typing.Optional[PositionOrientation],
        goal_pos: typing.Optional[PositionOrientation],
    ):
        if start_pos is not None:
            self._start_pos = start_pos
            self.move_robot_to_pos(start_pos)

            if self._robot.record_data_dir is not None:
                self.node.set_parameter(
                    rclpy.parameter.Parameter(
                        self.namespace("start"),
                        rclpy.Parameter.Type.DOUBLE_ARRAY,
                        [float(v) for v in self._start_pos]
                    )
                )

        if goal_pos is not None:
            self._goal_pos = goal_pos
            self._publish_goal(self._goal_pos)

            if self._robot.record_data_dir is not None:
                self.node.set_parameter(
                    rclpy.parameter.Parameter(
                        self.namespace("goal"),
                        rclpy.Parameter.Type.DOUBLE_ARRAY,
                        [float(v) for v in self._goal_pos]
                    )
                )

        if self._clear_costmaps_srv.wait_for_service(timeout_sec=1.0):
            self._clear_costmaps_srv.call_async(std_srvs.Empty.Request())

        return self._position, self._goal_pos

    @property
    def _is_goal_reached(self) -> bool:
        start = self._position
        goal = self._goal_pos

        distance_to_goal: float = np.linalg.norm(
            np.array(goal[:2]) - np.array(start[:2])
        )

        angle_to_goal: float = np.pi - \
            np.abs(np.abs(goal[2] - start[2]) - np.pi)

        return (
            distance_to_goal < self._goal_tolerance_distance
            and angle_to_goal < self._goal_tolerance_angle
        )

    def _publish_goal_periodically(self, *args, **kwargs):
        if self._goal_pos is not None:
            self._publish_goal(self._goal_pos)

    def _publish_goal(self, goal: PositionOrientation):
        goal_msg = geometry_msgs.PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal.x
        goal_msg.pose.position.y = goal.y
        goal_msg.pose.position.z = 0.

        goal_msg.pose.orientation = orientation = geometry_msgs.Quaternion()
        orientation.x, orientation.y, orientation.z, orientation.w = quaternion_from_euler(
            0.0, 0.0, goal.orientation, axes="sxyz")

        self._move_base_goal_pub.publish(goal_msg)

    def _launch_robot(self):
        self.node.get_logger().warn(f"START WITH MODEL {self.namespace}")

        if Utils.get_arena_type() != Constants.ArenaType.TRAINING:
            launch_description = launch.LaunchDescription()

            launch_arguments = {
                'SIMULATOR': Utils.get_simulator().value,
                'model': self.model_name,
                'name': self.name,
                'namespace': self.namespace,
                'frame': f"{self.name}/" if self.name else '',
                'inter_planner': self._robot.inter_planner,
                'local_planner': self._robot.local_planner,
                # 'complexity': self.node.declare_parameter('complexity', 1).value,
                # 'train_mode': self.node.declare_parameter('train_mode', False).value,
                'agent_name': self._robot.agent,
            }

            if self._robot.record_data_dir:
                launch_arguments.update({
                    'record_data': 'true',
                    'record_data_dir': self._robot.record_data_dir,
                })

            launch_description.add_action(
                launch.actions.IncludeLaunchDescription(
                    launch.launch_description_sources.PythonLaunchDescriptionSource(
                        os.path.join(
                            ament_index_python.packages.get_package_share_directory(
                                'arena_bringup'),
                            'launch/testing/robot.launch.py'
                        )
                    ),
                    launch_arguments=launch_arguments.items(),
                )
            )

            self.launch_service = launch.launch_service.LaunchService()
            self.launch_service.include_launch_description(launch_description)
            self.launch_service.run()

        # TODO
        # base_frame: str = self.node.get_parameter_or(self.namespace("robot_base_frame"), DefaultParameter('')).value
        # sensor_frame: str = self.node.get_parameter_or(self.namespace("robot_sensor_frame"), DefaultParameter('')).value

        # self.node.set_parameters([
        #     Parameter(self.namespace("move_base", "global_costmap", "robot_base_frame"),
        #               Parameter.Type.STRING, os.path.join(self.name, base_frame)),
        #     Parameter(self.namespace("move_base", "local_costmap", "robot_base_frame"),
        #               Parameter.Type.STRING, os.path.join(self.name, base_frame)),
        #     Parameter(self.namespace("move_base", "local_costmap", "scan", "sensor_frame"),
        #               Parameter.Type.STRING, os.path.join(self.name, sensor_frame)),
        #     Parameter(self.namespace("move_base", "global_costmap", "scan", "sensor_frame"),
        #               Parameter.Type.STRING, os.path.join(self.name, sensor_frame))
        # ])

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
