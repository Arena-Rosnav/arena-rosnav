import dataclasses
import os
import traceback
import typing
from typing import Dict, List


import rclpy
import rclpy.executors
import rclpy.node
import rclpy.callback_groups
import rclpy.parameter

import std_srvs.srv as std_srvs
import yaml
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Empty, Int16
from std_srvs.srv import Empty as EmptySrv

import launch
import task_generator.utils.arena as Utils
from task_generator import NodeInterface
from task_generator.constants import Constants
from task_generator.constants.runtime import Configuration
from task_generator.manager.entity_manager import (EntityManager,
                                                   EntityManagerRegistry)
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.manager.world_manager import WorldManager
from task_generator.shared import ModelWrapper, Namespace, Robot, gen_init_pos
from task_generator.simulators import BaseSimulator, SimulatorRegistry
from task_generator.tasks import Task
from task_generator.tasks.task_factory import TaskFactory
from task_generator.utils import ModelLoader
from task_generator.utils.ros_params import ROSParamServer


def create_default_robot_list(
    robot_model: ModelWrapper,
    name: str,
    inter_planner: str,
    local_planner: str,
    agent: str

) -> List[Robot]:
    return [
        Robot(
            model=robot_model,
            inter_planner=inter_planner,
            local_planner=local_planner,
            agent=agent,
            position=next(gen_init_pos),
            name=name,
            # TODO record_data_dir must be added to TASKGEN_NODE
            # record_data_dir=self.declare_parameter(
            #     'record_data_dir', None).value,
            extra=dict(),
        )
    ]


def read_robot_setup_file(setup_file: str) -> List[Dict]:
    try:
        with open(
            os.path.join(
                get_package_share_directory("arena_bringup"),
                "configs",
                "robot_setup",
                setup_file,
            ),
            "r",
        ) as f:
            robots: List[Dict] = yaml.safe_load(f)["robots"]

        return robots

    except BaseException:
        traceback.print_exc()
        raise Exception("Failed to read robot setup file")


class TaskGenerator(NodeInterface.Taskgen_T):
    """
    Task Generator Node
    Will initialize and reset all tasks. The task to use is read from the `/task_mode` param.
    """

    _world_manager: WorldManager
    _entity_manager: EntityManager
    _simulator: BaseSimulator

    _initialized: bool

    do_launch: typing.Callable[[launch.LaunchDescription], None]

    def __init__(
        self,
        namespace: str = "",
        *,
        do_launch: typing.Callable[[launch.LaunchDescription], None]

    ):
        rclpy.node.Node.__init__(self, 'task_generator')
        ROSParamServer.__init__(self)
        self.conf = Configuration(self)

        self.do_launch = do_launch
        self._namespace = Namespace(namespace)

        # # Declare all parameters
        # self.declare_parameters(
        #     namespace='',
        #     parameters=[
        #         ('auto_reset', True),
        #         ('robot', 'jackal'),
        #         ('train_mode', False),
        #         ('robot_setup_file', ''),
        #         ('inter_planner', ''),
        #         ('local_planner', ''),
        #         ('agent_name', ''),
        #         ('robot_names', []),
        #         ('task_generator_setup_finished', False),
        #     ]
        # )

        Task.declare_parameters(self)

        self._auto_reset = self.rosparam[bool].get('auto_reset', True)
        self._train_mode = self.rosparam[bool].get('train_mode', False)

        # Publishers
        if not self._train_mode:
            self._pub_scenario_reset = self.create_publisher(
                Int16, 'scenario_reset', 1)
            self._pub_scenario_finished = self.create_publisher(
                Empty, 'scenario_finished', 10)

            # Services
            self.create_service(EmptySrv, 'reset_task',
                                self._reset_task_srv_callback)

        self._initialized = False
        self.create_timer(
            0.5,
            self._check_task_status,
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),
        )

    def _initialize(self):
        # Vars
        self._simulator = SimulatorRegistry.get(self.conf.Arena.SIMULATOR.value)(
            namespace=self._namespace
        )

        self._robot_loader = ModelLoader(
            os.path.join(
                Utils.get_simulation_setup_path(),
                'entities',
                'robots',
            ),
        )

        self._start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self._task = self._get_predefined_task()
        self.rosparam[list[str]].set(
            'robot_names', self._task.robot_names)

        self._number_of_resets = 0

        # self.srv_start_model_visualization = self.create_client(
        #     EmptySrv, 'start_model_visualization')
        # while not self.srv_start_model_visualization.wait_for_service(
        #         timeout_sec=1.0):
        #     self.get_logger().info('start_model_visualization service not available, waiting again...')
        # self.srv_start_model_visualization.call_async(EmptySrv.Request())

        self.reset_task(first_map=True)

        # try:
        #     self.set_parameters(
        #         [rclpy.Parameter('task_generator_setup_finished', value=True)])
        #     self.srv_setup_finished = self.create_client(
        #         EmptySrv, 'task_generator_setup_finished')
        #     while not self.srv_setup_finished.wait_for_service(
        #             timeout_sec=1.0):
        #         self.get_logger().info(
        #             'task_generator_setup_finished service not available, waiting again...')
        #     self.srv_setup_finished.call_async(EmptySrv.Request())
        # except BaseException:
        #     pass

        self._initialized = True

    def _get_predefined_task(self, **kwargs):
        """
        Gets the task based on the passed mode
        """
        if self._simulator is None:
            self._simulator = SimulatorRegistry.get(self.conf.Arena.SIMULATOR.value)(
                self._namespace
            )

        self._world_manager = WorldManager()

        self._entity_manager = EntityManagerRegistry.get(self.conf.Arena.ENTITY_MANAGER.value)(
            namespace=self._namespace,
            simulator=self._simulator,
        )

        obstacle_manager = ObstacleManager(
            namespace=self._namespace,
            world_manager=self._world_manager,
            simulator=self._simulator,
            entity_manager=self._entity_manager,
        )

        obstacle_manager.spawn_world_obstacles(self._world_manager.world)

        robot_managers = self._create_robot_managers()

        # For every robot
        # - Create a unique namespace name
        # - Create a robot manager
        # - Launch the robot.launch file

        tm_modules = self.conf.TaskMode.TM_MODULES.value
        tm_modules.append(Constants.TaskMode.TM_Module.CLEAR_FORBIDDEN_ZONES)
        tm_modules.append(Constants.TaskMode.TM_Module.RVIZ_UI)

        if self.conf.Arena.WORLD.value == "dynamic_map":
            tm_modules.append(Constants.TaskMode.TM_Module.DYNAMIC_MAP)

        self.get_logger().debug("utils calls task factory")
        task = TaskFactory.combine(tm_modules)(
            obstacle_manager=obstacle_manager,
            robot_managers=robot_managers,
            world_manager=self._world_manager,
            namespace=self._namespace,
            **kwargs,
        )

        return task

    def _create_robot_managers(self) -> List[RobotManager]:
        # Read robot setup file
        robot_setup_file: str = self.rosparam[str].get(
            'robot_setup_file',
            ''
        )
        robot: str = self.rosparam[str].get('robot', '')

        if robot_setup_file == "":
            robots = create_default_robot_list(
                robot_model=self._robot_loader.bind(robot),
                inter_planner=self.rosparam[str].get("inter_planner", ''),
                local_planner=self.rosparam[str].get("local_planner", ''),
                agent=self.rosparam[str].get("agent_name", ''),
                name=f"{self._namespace[1:]}_{robot}"
                if self._train_mode
                else robot,
            )
        else:
            robots = [
                dataclasses.replace(
                    Robot.parse(
                        robot,
                        model=self._robot_loader.bind(robot["model"]),
                    ),
                    name=f'{robot["model"]}_{i}_{robot.get("amount", 1) - 1}'
                )
                for robot in read_robot_setup_file(robot_setup_file)
                for i in range(robot.get("amount", 1))
            ]

        if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
            return [
                RobotManager(
                    namespace=self._namespace,
                    entity_manager=self._entity_manager,
                    robot=robots[0],
                )
            ]

        robot_managers: List[RobotManager] = []

        for robot in robots:
            robot_managers.append(
                # RobotManager(os.path.join(namespace, name), simulator, robot)
                # old but working due to namespace issue with "/" prefix in
                # robot name
                RobotManager(
                    namespace=self._namespace,
                    entity_manager=self._entity_manager,
                    robot=robot,
                )
            )

        return robot_managers

    # RUNTIME
    def reset_task(self, **kwargs):
        self._start_time = self.get_clock().now().seconds_nanoseconds()[0]

        self._simulator.before_reset_task()

        self.get_logger().info("resetting")

        self._task.reset(callback=lambda: False, **kwargs)

        self._pub_scenario_reset.publish(Int16(data=self._number_of_resets))
        self._number_of_resets += 1
        self._send_end_message_on_end()

        self._simulator.after_reset_task()

        self.get_logger().info("=============")
        self.get_logger().info("Task Reset!")
        self.get_logger().info("=============")

    def _check_task_status(self, *args, **kwargs):
        if not self._initialized:
            return self._initialize()
        if self._task.is_done:
            self.reset_task()

    def _reset_task_srv_callback(
            self, request: std_srvs.Empty.Request, response: std_srvs.Empty.Response):
        self.get_logger().debug("Task Generator received task-reset request!")
        self.reset_task()
        return response

    def _send_end_message_on_end(self):
        if self.conf.General.DESIRED_EPISODES.value < 0 or self._number_of_resets < self.conf.General.DESIRED_EPISODES.value:
            return

        self.get_logger().info(
            f"Shutting down. All {int(self.conf.General.DESIRED_EPISODES.value)} tasks completed")
        rclpy.shutdown()
