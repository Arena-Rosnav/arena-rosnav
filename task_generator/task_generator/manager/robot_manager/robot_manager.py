import dataclasses
import os
import typing

import numpy as np
import scipy.spatial.transform

import rclpy
import rclpy.publisher
import rclpy.timer
import rclpy.client

from task_generator import NodeInterface
import task_generator.utils.arena as Utils
from task_generator.utils.geometry import angle_diff, quaternion_from_euler
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
    _goal_pub: rclpy.publisher.Publisher
    _pub_goal_timer: rclpy.timer.Timer
    _clear_costmaps_srv: rclpy.client.Client

    _rate_setup: rclpy.timer.Rate

    @property
    def robot(self) -> Robot:
        return self._robot

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
        self._rate_setup = self.node.create_rate(.1)

        self._namespace = namespace
        self._entity_manager = entity_manager
        self._start_pos = PositionOrientation(0, 0, 0)
        self._goal_pos = PositionOrientation(0, 0, 0)

        # Parameter handling
        try:
            self._goal_tolerance_distance = self.node.conf.Robot.GOAL_TOLERANCE_RADIUS.value
            self._goal_tolerance_angle = self.node.conf.Robot.GOAL_TOLERANCE_ANGLE.value
            self._safety_distance = self.node.conf.Robot.SPAWN_ROBOT_SAFE_DIST.value
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

        _gen_goal_topic = self.namespace("goal_pose")

        self._goal_pub = self.node.create_publisher(
            geometry_msgs.PoseStamped, _gen_goal_topic, 10
        )

        self.node.create_subscription(
            nav_msgs.Odometry, self.namespace(
                "odom"), self._robot_pos_callback, 10
        )

        self._launch_robot()

        self._robot_radius = self.node.rosparam[float].get(
            'robot_radius',
            0.25
        )

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

    def frame(self) -> str:
        return self._robot.frame

    @property
    def namespace(self) -> Namespace:
        if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
            return Namespace(
                f"{self._namespace}{self._namespace}_{self.model_name}"
            )

        return self._namespace(self.name)

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
                self.node.rosparam[list[float]].set(
                    self.namespace("start"),
                    [self.start_pos.x, self.start_pos.y, self.start_pos.orientation]
                )

        if goal_pos is not None:
            self._goal_pos = goal_pos
            self._publish_goal(self._goal_pos)

            if self._robot.record_data_dir is not None:
                self.node.rosparam[list[float]].set(
                    self.namespace("goal"),
                    [self.goal_pos.x, self.goal_pos.y,
                        self.goal_pos.orientation]
                )

        # TODO
        # if self._clear_costmaps_srv.wait_for_service(timeout_sec=1.0):
        #     self._clear_costmaps_srv.call_async(std_srvs.Empty.Request())

        return self._position, self._goal_pos

    @property
    def _is_goal_reached(self) -> bool:
        start = self._position
        goal = self._goal_pos

        distance_to_goal: float = float(np.linalg.norm(
            np.array([goal.x, goal.y]) - np.array([start.x, start.y])
        ).flat[0])

        angle_to_goal: float = angle_diff(goal.orientation, start.orientation)

        return (
            distance_to_goal < self._goal_tolerance_distance
            and angle_to_goal < self._goal_tolerance_angle
        )

    def _publish_goal(self, goal: PositionOrientation):
        goal_msg = geometry_msgs.PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal.x
        goal_msg.pose.position.y = goal.y
        goal_msg.pose.position.z = 0.

        goal_msg.pose.orientation = orientation = geometry_msgs.Quaternion()
        orientation.x, orientation.y, orientation.z, orientation.w = quaternion_from_euler(
            0.0, 0.0, goal.orientation, axes="sxyz")

        self._goal_pub.publish(goal_msg)

    def _launch_robot(self):
        self.node.get_logger().warn(f"START WITH MODEL {self.name}")

        if Utils.get_arena_type() != Constants.ArenaType.TRAINING:
            launch_description = launch.LaunchDescription()

            launch_arguments = {
                'robot': self.model_name,
                # 'world_path': self.node.conf.Arena.get_world_path(),
                # 'simulator': self.node.conf.Arena.SIMULATOR.value.value,
                # 'name': self.name,
                'namespace': self.namespace,
                # 'use_namespace': 'True',
                'frame': self._robot.frame,
                'inter_planner': self._robot.inter_planner,
                'global_planner': self._robot.global_planner,
                'local_planner': self._robot.local_planner,
                # 'complexity': self.node.declare_parameter('complexity', 1).value,
                # 'train_mode': self.node.declare_parameter('train_mode', False).value,
                'agent_name': self._robot.agent,
                'use_sim_time': 'True',
            }

            if self._robot.record_data_dir:
                launch_arguments.update({
                    'record_data': 'True',
                    'record_data_dir': self._robot.record_data_dir,
                })

            launch_description.add_action(
                launch.actions.IncludeLaunchDescription(
                    launch.launch_description_sources.PythonLaunchDescriptionSource(
                        os.path.join(
                            ament_index_python.packages.get_package_share_directory('arena_simulation_setup'),
                            'launch/robot.launch.py'
                        )
                    ),
                    launch_arguments=launch_arguments.items(),
                )
            )
            self.node.do_launch(launch_description)

            while 'bt_navigator' not in self.node.get_node_names():
                # TODO redo this globally in the robots manager, every get_node_names call is expensive
                self._rate_setup.sleep()  # we love race conditions

        # TODO
        # base_frame: str = self.node.get_parameter_or(self.namespace("robot_base_frame"), DefaultParameter('')).value
        # sensor_frame: str =
        # self.node.get_parameter_or(self.namespace("robot_sensor_frame"),
        # DefaultParameter('')).value

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

    def update(self):
        """
        Live-update some kwargs of robot
        """
        # TODO implement record data dir

    def destroy(self):
        """
        Destroy robot and remove from simulation and navigation stack.
        """
        self._entity_manager.remove_robot(self.name)
        # TODO kill node in navigation stack
