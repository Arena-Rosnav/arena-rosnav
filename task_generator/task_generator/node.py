import os
import traceback
import typing

import arena_simulation_setup
import launch
import rclpy
import rclpy.callback_groups
import std_srvs.srv as std_srvs
import task_generator_msgs.srv
import yaml
from ament_index_python.packages import get_package_share_directory
from arena_rclpy_mixins.shared import Namespace
from std_msgs.msg import Empty, Int16
from std_srvs.srv import Empty as EmptySrv

from task_generator.constants import Constants
from task_generator.constants.runtime import Configuration
from task_generator.manager.entity_manager import (EntityManager,
                                                   EntityManagerRegistry)
from task_generator.manager.environment_manager import EnvironmentManager
from task_generator.manager.robot_manager import RobotsManagerROS
from task_generator.manager.robot_manager.robots_manager_ros import \
    RobotsManager
from task_generator.manager.world_manager.world_manager_ros import \
    WorldManagerROS as WorldManager
from task_generator.simulators import BaseSimulator, SimulatorRegistry
from task_generator.tasks import Task
from task_generator.tasks.task_factory import TaskFactory

from . import NodeInterface


def read_robot_setup_file(setup_file: str) -> list[dict]:
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
            robots: list[dict] = yaml.safe_load(f)["robots"]

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
    _environment_manager: EnvironmentManager
    _robots_manager: RobotsManager
    _simulator: BaseSimulator

    _initialized: bool

    do_launch: typing.Callable[[launch.LaunchDescription], None]

    def __init__(
        self,
        namespace: str = "task_generator_node",
        *,
        do_launch: typing.Callable[[launch.LaunchDescription], None]

    ):
        super().__init__('task_generator')
        self.conf = Configuration(self)

        self.do_launch = do_launch
        self._namespace = Namespace(namespace)

        Task.declare_parameters(self)

        self._auto_reset = self.rosparam[bool].get('auto_reset', True)
        self._train_mode = self.rosparam[bool].get('train_mode', False)

        # Publishers
        self._pub_scenario_reset = self.create_publisher(Int16, 'scenario_reset', 1)
        self._pub_scenario_finished = self.create_publisher(Empty, 'scenario_finished', 10)

        self._set_up_services()

        self._initialized = False
        self.create_timer(
            0.5,
            self._check_task_status,
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),
        )

    def _initialize(self):
        self._start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self._task = self._get_predefined_task()

        self._number_of_resets = 0

        self.reset_task(first_map=True)

        self._initialized = True
        self.rosparam[bool].set('initialized', True)

    def _get_predefined_task(self, **kwargs):
        """
        Gets the task based on the passed mode
        """
        self._simulator = SimulatorRegistry.get(self.conf.Arena.SIMULATOR.value)(
            self._namespace
        )

        self._entity_manager = EntityManagerRegistry.get(self.conf.Arena.ENTITY_MANAGER.value)(
            namespace=self._namespace,
            simulator=self._simulator,
        )

        self._environment_manager = EnvironmentManager(
            namespace=self._namespace,
            simulator=self._simulator,
            entity_manager=self._entity_manager,
        )

        self._world_manager = WorldManager(
            environment_manager=self._environment_manager
        )

        def on_world_change():
            self._environment_manager.reset()
            self._environment_manager.spawn_world_obstacles(self._world_manager.world)
        self._world_manager.on_world_change(on_world_change)
        self._world_manager.start()

        self._robots_manager = RobotsManagerROS(self._environment_manager)

        tm_modules = self.conf.TaskMode.TM_MODULES.value
        tm_modules.add(Constants.TaskMode.TM_Module.CLEAR_FORBIDDEN_ZONES)
        tm_modules.add(Constants.TaskMode.TM_Module.RVIZ_UI)

        if self.conf.Arena.WORLD.value == "dynamic_map":
            tm_modules.add(Constants.TaskMode.TM_Module.DYNAMIC_MAP)

        self.get_logger().debug("utils calls task factory")
        return TaskFactory.combine(list(tm_modules))(
            environment_manager=self._environment_manager,
            robots_manager=self._robots_manager,
            world_manager=self._world_manager,
            namespace=self._namespace,
            **kwargs,
        )

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

    def _send_end_message_on_end(self):
        if self.conf.General.DESIRED_EPISODES.value < 0 or self._number_of_resets < self.conf.General.DESIRED_EPISODES.value:
            return

        self.get_logger().info(
            f"Shutting down. All {int(self.conf.General.DESIRED_EPISODES.value)} tasks completed")
        rclpy.shutdown()

    # SERVICES
    def _cb_reset_task(
        self,
        request: std_srvs.Empty.Request,
        response: std_srvs.Empty.Response
    ):
        self.get_logger().debug("Task Generator received task-reset request!")
        self.reset_task()
        return response

    def _cb_get_configs_environments(
        self,
        request: task_generator_msgs.srv.GetEnvironments.Request,
        response: task_generator_msgs.srv.GetEnvironments.Response,
    ):
        response.environments = arena_simulation_setup.Environment.list()
        response.environments.sort()
        return response

    def _cb_get_configs_parametrized(
        self,
        request: task_generator_msgs.srv.GetParametrizeds.Request,
        response: task_generator_msgs.srv.GetParametrizeds.Response,
    ):
        response.parametrizeds = arena_simulation_setup.Parametrized.list()
        response.parametrizeds.sort()
        return response
    
    def _cb_get_randoms(
        self,
        request: task_generator_msgs.srv.GetRandoms.Request,
        response: task_generator_msgs.srv.GetRandoms.Response,
    ):
        response.n_static_obstacles = [5, 15]
        response.n_interactive_obstacles = [0, 0]
        response.n_dynamic_obstacles = [1, 5]

        response.models_static_obstacles = arena_simulation_setup.Obstacle(self._world_manager.world_name).list()
        response.models_static_obstacles.sort()
        response.models_interactive_obstacles = []
        response.models_interactive_obstacles.sort()
        response.models_dynamic_obstacles = arena_simulation_setup.DynamicObstacle(self._world_manager.world_name).list()
        response.models_dynamic_obstacles.sort()

        return response
    
    def _cb_get_scenarios(
        self,
        request: task_generator_msgs.srv.GetScenarios.Request,
        response: task_generator_msgs.srv.GetScenarios.Response,
    ):
        response.scenarios = arena_simulation_setup.World(
            request.world or self._world_manager.world_name
        ).scenarios
        response.scenarios.sort()
        return response
    
    def _cb_get_worlds(
        self,
        request: task_generator_msgs.srv.GetWorlds.Request,
        response: task_generator_msgs.srv.GetWorlds.Response,
    ):
        response.worlds = arena_simulation_setup.World.list()
        response.worlds.sort()
        return response
    
    def _cb_get_robots(
        self,
        request: task_generator_msgs.srv.GetRobots.Request,
        response: task_generator_msgs.srv.GetRobots.Response,
    ):
        response.robots = arena_simulation_setup.Robot.list()
        response.robots.sort()
        return response

    def _set_up_services(self):
        # Services
        self.create_service(
            EmptySrv,
            self.service_namespace('reset_task'),
            self._cb_reset_task
        )

        self.create_service(
            task_generator_msgs.srv.GetEnvironments,
            self.service_namespace('get_environments'),
            self._cb_get_configs_environments
        )

        self.create_service(
            task_generator_msgs.srv.GetParametrizeds,
            self.service_namespace('get_parametrizeds'),
            self._cb_get_configs_parametrized
        )

        self.create_service(
            task_generator_msgs.srv.GetRandoms,
            self.service_namespace('get_randoms'),
            self._cb_get_randoms
        )

        self.create_service(
            task_generator_msgs.srv.GetScenarios,
            self.service_namespace('get_scenarios'),
            self._cb_get_scenarios
        )

        self.create_service(
            task_generator_msgs.srv.GetRobots,
            self.service_namespace('get_robots'),
            self._cb_get_robots
        )

        self.create_service(
            task_generator_msgs.srv.GetWorlds,
            self.service_namespace('get_worlds'),
            self._cb_get_worlds
        )