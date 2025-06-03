import os
import typing

import action_msgs.msg
import ament_index_python
import arena_bringup.extensions.NodeLogLevelExtension as NodeLogLevelExtension
import attrs
import geometry_msgs.msg as geometry_msgs
import launch
import lifecycle_msgs.msg
import nav_msgs.msg as nav_msgs
import rclpy
import rclpy.client
import rclpy.publisher
import rclpy.timer
import scipy.spatial.transform
import task_generator.utils.arena as Utils
from arena_rclpy_mixins.shared import Namespace
from nav2_msgs.srv import ClearCostmapAroundRobot
from task_generator import NodeInterface
from task_generator.constants import Constants
from task_generator.manager.entity_manager import EntityManager
from task_generator.manager.entity_manager.utils import YAMLUtil
from task_generator.manager.environment_manager import EnvironmentManager
from task_generator.shared import ModelType, PositionOrientation, Robot


class RobotManager(NodeInterface):
    """
    The robot manager manages the goal and start
    position of a robot for all task modes.
    """

    _namespace: Namespace
    _entity_manager: EntityManager
    _environment_manager: EnvironmentManager
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
    _is_goal_reached: bool
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
        self,
        namespace: Namespace,
        entity_manager: EntityManager,
        environment_manager: EnvironmentManager,
        robot: Robot,
    ):
        NodeInterface.__init__(self)
        self._rate_setup = self.node.create_rate(.1)

        self._namespace = namespace
        self._entity_manager = entity_manager
        self._environment_manager = environment_manager

        self._start_pos = PositionOrientation(0, 0, 0)
        self._goal_pos = PositionOrientation(0, 0, 0)
        self._is_goal_reached = False

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
        self._robot.extra.setdefault('namespace', self.namespace)
        self._position = self._start_pos
        self._goal_timer = None

    def set_up_robot(self):
        self._robot = self._environment_manager.spawn_robot(
            attrs.evolve(
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
                )
            )
        )

        _gen_goal_topic = self.namespace("goal_pose")

        self._goal_pub = self.node.create_publisher(
            geometry_msgs.PoseStamped,
            _gen_goal_topic,
            10,
        )

        self.node.create_subscription(
            nav_msgs.Odometry,
            self.namespace("odom"),
            self._robot_pos_callback,
            10
        )

        self.node.create_subscription(
            action_msgs.msg.GoalStatusArray,
            self.namespace('navigate_to_pose', '_action', 'status'),
            self._goal_status_callback,
            1
        )

        self._launch_robot()

        self._robot_radius = self.node.rosparam[float].get(
            'robot_radius',
            0.25
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
        self.clearCostmapAroundRobot(5.0)

    def clearCostmapAroundRobot(self, reset_distance: float) -> bool:
        """Clear the costmap around the robot."""

        state = self.node.get_lifecycle_state(
            node_name := self.node.service_namespace(self.name, 'local_costmap/local_costmap'),
        )
        if state.id != lifecycle_msgs.msg.State.PRIMARY_STATE_ACTIVE:
            return False

        service_name = os.path.abspath(node_name('../clear_around_local_costmap'))

        self._logger.info(f"Service name: {service_name}")
        self._clear_costmaps_srv = self.node.create_client(
            ClearCostmapAroundRobot,
            service_name,
        )
        while not self._clear_costmaps_srv.wait_for_service(timeout_sec=1.0):
            self._logger.warn(f'{service_name} service not available, waiting...')
        req = ClearCostmapAroundRobot.Request()
        req.reset_distance = reset_distance

        result = self._clear_costmaps_srv.call(req)
        if result is None:
            self._logger.error(
                f"service call failed for {service_name}")
            return False
        self._logger.info(
            f"successfull service call for {service_name}"
        )
        return True

    def reset(
        self,
        start_pos: typing.Optional[PositionOrientation],
        goal_pos: typing.Optional[PositionOrientation],
    ):
        if start_pos is not None:
            self._start_pos = self._environment_manager.realize(start_pos)
            self.move_robot_to_pos(start_pos)

            if self._robot.record_data_dir:
                self.node.rosparam[list[float]].set(
                    self.namespace.robot_ns.ParamNamespace()("start"),
                    [self.start_pos.x, self.start_pos.y, self.start_pos.orientation]
                )
        if goal_pos is not None:
            self._goal_pos = self._environment_manager.realize(goal_pos)
            self._publish_goal(self._goal_pos)

            if self._robot.record_data_dir:
                self.node.rosparam[list[float]].set(
                    self.namespace.robot_ns.ParamNamespace()("goal"),
                    [self.goal_pos.x, self.goal_pos.y,
                        self.goal_pos.orientation]
                )
        return self._position, self._goal_pos

    def _publish_goal_callback(self):
        from geometry_msgs.msg import PoseStamped
        current_time = self.node.get_clock().now().nanoseconds / 1e9
        if (current_time - self._goal_start_time) >= 60.0:
            self._logger.info("Goal publishing duration reached, stopping")
            if self._goal_timer is not None:
                self._goal_timer.cancel()
                self._goal_timer.destroy()
                self._goal_timer = None
            return

        if self._is_goal_reached:
            self._logger.info("Goal reached, stopping goal publication")
            if self._goal_timer is not None:
                self._goal_timer.cancel()
                self._goal_timer.destroy()
                self._goal_timer = None
            return

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose = self._goal_pos.to_pose()
        self._goal_pub.publish(goal_msg)

    def _publish_goal(self, goal: PositionOrientation):
        # only way to circumvent amcl absolutely trolling us is to create this loop
        from geometry_msgs.msg import PoseStamped
        self._logger.info(
            f"Publishing goal: x={goal.x}, y={goal.y}, orientation={goal.orientation}")

        self._goal_pos = goal

        if self._goal_timer is not None:
            self._goal_timer.cancel()
            self._goal_timer.destroy()

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose = goal.to_pose()
        self._goal_pub.publish(goal_msg)

        self._goal_start_time = self.node.get_clock().now().nanoseconds / 1e9
        self._goal_timer = self.node.create_timer(
            3.0,
            self._publish_goal_callback,
        )

    def _launch_robot(self):
        self._logger.warn(f"START WITH MODEL {self.name}")

        if Utils.get_arena_type() != Constants.ArenaType.TRAINING:

            launch_description = launch.LaunchDescription()
            current_log_level = rclpy.logging.get_logger_effective_level(self.node.get_logger().name).name.lower()
            launch_description.add_action(NodeLogLevelExtension.SetGlobalLogLevelAction(current_log_level))

            launch_arguments = {
                'robot': self.model_name,
                # 'simulator': self.node.conf.Arena.SIMULATOR.value.value,
                # 'name': self.name,
                'task_generator_node': os.path.join(self.node.get_namespace(), self.node.get_name()),
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
                'amcl': 'true' if self.node.conf.Arena.SIMULATOR.value == Constants.Simulator.GAZEBO else 'false',
            }

            if self._robot.record_data_dir:
                launch_arguments.update({
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

            while 'bt_navigator' not in (node_names := self.node.get_node_names()):
                self._logger.debug(f'waiting for bt_navigator in {node_names}')
                # TODO redo this globally in the robots manager, every get_node_names call is expensive
                self._rate_setup.sleep()  # we love race conditions

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

    def _goal_status_callback(self, data: action_msgs.msg.GoalStatusArray):
        last_goal = next(reversed(data.status_list), None)
        self._is_goal_reached = last_goal and last_goal.status == action_msgs.msg.GoalStatus.STATUS_SUCCEEDED

    def update(self):
        """
        Live-update some kwargs of robot
        """
        # TODO implement record data dir

    def destroy(self):
        """
        Destroy robot and remove from simulation and navigation stack.
        """
        if self._goal_timer is not None:
            self._goal_timer.cancel()
            self._goal_timer.destroy()
        self._entity_manager.remove_robot(self.name)
        # TODO kill node in navigation stack
