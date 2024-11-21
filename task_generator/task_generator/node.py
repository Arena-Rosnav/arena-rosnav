import dataclasses
import os
import traceback
from typing import Dict, List

import numpy as np

import rclpy
import rclpy.node
import rclpy.executors
from rclpy.parameter import Parameter

import yaml
from ament_index_python.packages import get_package_share_directory
from task_generator.constants import Constants
from task_generator.constants.runtime import Configuration


from task_generator.simulators import BaseSimulator, SimulatorRegistry
from task_generator.manager.entity_manager import EntityManagerRegistry
from task_generator.manager.world_manager import WorldManager
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.manager.utils import WorldMap
from task_generator.shared import (
    ModelWrapper,
    Namespace,
    Robot,
    gen_init_pos
)

from task_generator.tasks import Task
from task_generator.tasks.task_factory import TaskFactory
from task_generator.utils import ModelLoader
import task_generator.utils.arena as Utils

from task_generator.manager.world_manager import WorldManager
from task_generator.manager.obstacle_manager import ObstacleManager

from std_msgs.msg import Int16, Empty
import std_srvs.srv as std_srvs
from std_srvs.srv import Empty as EmptySrv
import nav_msgs.msg as nav_msgs
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


class TaskGenerator(ROSParamServer, rclpy.node.Node):
    """
    Task Generator Node
    Will initialize and reset all tasks. The task to use is read from the `/task_mode` param.
    """

    def __init__(self, namespace: str = ""):
        rclpy.node.Node.__init__(self, 'task_generator')
        ROSParamServer.__init__(self)
        self.Configuration = Configuration(self)
        self._namespace = Namespace(namespace)

        # Declare all parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('auto_reset', True),
                ('robot', 'jackal'),
                ('train_mode', False),
                ('robot_setup_file', ''),
                ('inter_planner', ''),
                ('local_planner', ''),
                ('agent_name', ''),
                ('robot_names', []),
                ('task_generator_setup_finished', False),
            ]
        )

        Task.declare_parameters(self)

        self._auto_reset = self.get_parameter('auto_reset').value
        self._train_mode = self.get_parameter('train_mode').value

        # Publishers
        if not self._train_mode:
            self._pub_scenario_reset = self.create_publisher(
                Int16, 'scenario_reset', 1)
            self._pub_scenario_finished = self.create_publisher(
                Empty, 'scenario_finished', 10)

            # Services
            self.create_service(EmptySrv, 'reset_task',
                                self._reset_task_srv_callback)

    def post_init(self):
        # Vars
        self._env_wrapper = SimulatorRegistry.get(Utils.get_simulator())(
            namespace=self._namespace
        )

        # Loaders
        workspace_root = ModelLoader.getArenaDir()

        self._robot_loader = ModelLoader(
            os.path.join(workspace_root, 'src', 'arena',
                         'simulation-setup', 'entities', 'robots')
        )

        if not self._train_mode:
            self._start_time = self.get_clock().now().seconds_nanoseconds()[0]
            self._task = self._get_predefined_task()
            self.set_parameters(
                [rclpy.Parameter('robot_names', value=self._task.robot_names)])

            self._number_of_resets = 0

            # self.srv_start_model_visualization = self.create_client(
            #     EmptySrv, 'start_model_visualization')
            # while not self.srv_start_model_visualization.wait_for_service(
            #         timeout_sec=1.0):
            #     self.get_logger().info('start_model_visualization service not available, waiting again...')
            # self.srv_start_model_visualization.call_async(EmptySrv.Request())

            self.create_timer(0.5, self._check_task_status)

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

    def _get_predefined_task(self, **kwargs):
        """
        Gets the task based on the passed mode
        """
        if self._env_wrapper is None:
            self._env_wrapper = SimulatorRegistry.get(Utils.get_simulator())(
                self._namespace
            )

        # cli = self.create_client(map_distance_server_srvs.GetDistanceMap, '/distance_map')
        # while not cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')

        # req = map_distance_server_srvs.GetDistanceMap.Request()
        # future = cli.call_async(req)
        # rclpy.spin_until_future_complete(self, future)
        # map_response = future.result()

        DUMMY_MAP_SHAPE = (200, 200)
        DUMMY_MAP = nav_msgs.OccupancyGrid(
            info=nav_msgs.MapMetaData(
                height=DUMMY_MAP_SHAPE[0],
                width=DUMMY_MAP_SHAPE[1],
                resolution=0.1,
            ),
            data=list(
                np.pad(
                    np.zeros(
                        (DUMMY_MAP_SHAPE[0] - 2, DUMMY_MAP_SHAPE[1] - 2),
                        dtype=int,
                    ),
                    ((1, 1), (1, 1)),
                    mode='constant',
                    constant_values=1
                ).flat
            )
        )

        world_manager = WorldManager(
            world_map=WorldMap.from_occupancy_grid(occupancy_grid=DUMMY_MAP)
        )

        self._entity_manager = EntityManagerRegistry.get(Utils.get_entity_manager())(
            namespace=self._namespace,
            simulator=self._env_wrapper,
        )

        obstacle_manager = ObstacleManager(
            namespace=self._namespace,
            world_manager=world_manager,
            simulator=self._env_wrapper,
            entity_manager=self._entity_manager,
        )

        obstacle_manager.spawn_world_obstacles(world_manager.world)

        robot_managers = self._create_robot_managers()

        # For every robot
        # - Create a unique namespace name
        # - Create a robot manager
        # - Launch the robot.launch file

        tm_modules = self.Configuration.TaskMode.TM_MODULES.value
        tm_modules.append(Constants.TaskMode.TM_Module.CLEAR_FORBIDDEN_ZONES)
        # tm_modules.append(Constants.TaskMode.TM_Module.RVIZ_UI)

        if self.declare_parameter("map_file", "").value == "dynamic_map":
            tm_modules.append(Constants.TaskMode.TM_Module.DYNAMIC_MAP)

        self.get_logger().debug("utils calls task factory")
        task = TaskFactory.combine(tm_modules)(
            obstacle_manager=obstacle_manager,
            robot_managers=robot_managers,
            world_manager=world_manager,
            namespace=self._namespace,
            **kwargs,
        )

        return task

    def _create_robot_managers(self) -> List[RobotManager]:
        # Read robot setup file
        robot_setup_file: str = self.get_parameter('robot_setup_file').value
        robot_model: str = self.get_parameter('robot').value

        if robot_setup_file == "":
            robots = create_default_robot_list(
                robot_model=self._robot_loader.bind(robot_model),
                inter_planner=self.get_parameter("inter_planner").value,
                local_planner=self.get_parameter("local_planner").value,
                agent=self.get_parameter("agent_name").value,
                name=f"{self._namespace[1:]}_{robot_model}"
                if self._train_mode
                else robot_model,
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

        self._env_wrapper.before_reset_task()

        self.get_logger().info("resetting")

        is_end = self._task.reset(callback=lambda: False, **kwargs)

        self._pub_scenario_reset.publish(Int16(data=self._number_of_resets))
        self._number_of_resets += 1
        self._send_end_message_on_end()

        self._env_wrapper.after_reset_task()

        self.get_logger().info("=============")
        self.get_logger().info("Task Reset!")
        self.get_logger().info("=============")

    def _check_task_status(self, *args, **kwargs):
        if self._task.is_done:
            self.reset_task()

    def _reset_task_srv_callback(
            self, request: std_srvs.Empty.Request, response: std_srvs.Empty.Response):
        self.get_logger().debug("Task Generator received task-reset request!")
        self.reset_task()
        return response

    def _send_end_message_on_end(self):
        if self.Configuration.General.DESIRED_EPISODES.value < 0 or self._number_of_resets < self.Configuration.General.DESIRED_EPISODES.value:
            return

        self.get_logger().info(
            f"Shutting down. All {int(self.Configuration.General.DESIRED_EPISODES.value)} tasks completed")
        rclpy.shutdown()
