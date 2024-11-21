import os
import typing

import rclpy

from task_generator import NodeInterface
from task_generator.constants import Constants
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.manager.world_manager import WorldManager
from task_generator.shared import DefaultParameter, PositionOrientation, rosparam_set
from task_generator.tasks import Task
from task_generator.tasks.modules import TM_Module
from task_generator.tasks.obstacles import TM_Obstacles
from task_generator.tasks.robots import TM_Robots

import std_msgs.msg as std_msgs
import rosgraph_msgs.msg as rosgraph_msgs
# import training.srv as training_srvs

from task_generator.utils import ModelLoader
from task_generator.constants.runtime import Configuration


class TaskFactory:
    registry_obstacles: typing.Dict[Constants.TaskMode.TM_Obstacles,
                                    typing.Callable[[], typing.Type[TM_Obstacles]]] = {}
    registry_robots: typing.Dict[Constants.TaskMode.TM_Robots,
                                 typing.Callable[[], typing.Type[TM_Robots]]] = {}
    registry_module: typing.Dict[Constants.TaskMode.TM_Module,
                                 typing.Callable[[], typing.Type[TM_Module]]] = {}

    @classmethod
    def register_obstacles(cls, name: Constants.TaskMode.TM_Obstacles):
        def inner_wrapper(
                loader: typing.Callable[[], typing.Type[TM_Obstacles]]):
            assert (
                name not in cls.registry_obstacles
            ), f"TaskMode '{name}' for obstacles already exists!"

            cls.registry_obstacles[name] = loader
            return loader

        return inner_wrapper

    @classmethod
    def register_robots(cls, name: Constants.TaskMode.TM_Robots):
        def inner_wrapper(loader: typing.Callable[[], typing.Type[TM_Robots]]):
            assert (
                name not in cls.registry_obstacles
            ), f"TaskMode '{name}' for robots already exists!"

            cls.registry_robots[name] = loader
            return loader

        return inner_wrapper

    @classmethod
    def register_module(cls, name: Constants.TaskMode.TM_Module):
        def inner_wrapper(loader: typing.Callable[[], typing.Type[TM_Module]]):
            assert (
                name not in cls.registry_obstacles
            ), f"TaskMode '{name}' for module already exists!"

            cls.registry_module[name] = loader
            return loader

        return inner_wrapper

    @classmethod
    def combine(cls, modules: typing.List[Constants.TaskMode.TM_Module] = [
    ]) -> typing.Type[Task]:
        for module in modules:
            assert (
                module in cls.registry_module
            ), f"Module '{module}' is not registered!"

        class CombinedTask(Task, NodeInterface):
            """
            Represents a combined task that involves multiple robots and obstacles.
            """

            PARAM_TM_ROBOTS = "tm_robots"
            PARAM_TM_OBSTACLES = "tm_obstacles"

            __param_tm_robots: Constants.TaskMode.TM_Robots
            __param_tm_obstacles: Constants.TaskMode.TM_Obstacles

            __tm_robots: TM_Robots
            __tm_obstacles: TM_Obstacles

            _force_reset: bool

            def __init__(
                self,
                obstacle_manager: ObstacleManager,
                robot_managers: typing.List[RobotManager],
                world_manager: WorldManager,
                namespace: str = "",
                *args,
                **kwargs,
            ):
                """
                Initializes a CombinedTask object.

                Args:
                    obstacle_manager (ObstacleManager): The obstacle manager for the task.
                    robot_managers (typing.List[RobotManager]): The typing.List of robot managers for the task.
                    world_manager (WorldManager): The world manager for the task.
                    namespace (str, optional): The namespace for the task. Defaults to "".
                    *args: Variable length argument typing.List.
                    **kwargs: Arbitrary keyword arguments.
                """
                NodeInterface.__init__(self)

                self._force_reset = False
                self.namespace = namespace

                self.obstacle_manager = obstacle_manager
                self.robot_managers = robot_managers
                self.world_manager = world_manager

                self.__modules = [
                    cls.registry_module[module]()(task=self) for module in modules
                ]

                self._train_mode = self.node.get_parameter_or(
                    "/train_mode", DefaultParameter(False)).value

                self.__reset_start = self.node.create_publisher(
                    std_msgs.Empty, 'reset_start', 1)
                self.__reset_end = self.node.create_publisher(
                    std_msgs.Empty, 'reset_end', 1)
                self.__reset_mutex = False

                self.node.create_subscription(
                    rosgraph_msgs.Clock, '/clock', self._clock_callback, 10)
                self.last_reset_time = 0
                self.clock = rosgraph_msgs.Clock()

                self.set_up_robot_managers()

                workspace_root = ModelLoader.getArenaDir()

                self.model_loader = ModelLoader(
                    os.path.join(
                        workspace_root,
                        'src',
                        'arena',
                        'simulation-setup',
                        'entities',
                        'obstacles',
                        'static')
                )
                self.dynamic_model_loader = ModelLoader(
                    os.path.join(
                        workspace_root,
                        'src',
                        'arena',
                        'simulation-setup',
                        'entities',
                        'obstacles',
                        'dynamic')
                )

                self.__param_tm_obstacles = None
                self.__param_tm_robots = None
                self.__modules = [
                    cls.registry_module[module]()(task=self) for module in modules
                ]

                if self._train_mode:
                    self.set_tm_robots(
                        Constants.TaskMode.TM_Robots(self.node.Configuration.TaskMode.TM_ROBOTS.value))
                    self.set_tm_obstacles(
                        Constants.TaskMode.TM_Obstacles(self.node.Configuration.TaskMode.TM_OBSTACLES.value))

            def set_tm_robots(self, tm_robots: Constants.TaskMode.TM_Robots):
                """
                Sets the task mode for robots.

                Args:
                    tm_robots (Constants.TaskMode.TM_Robots): The task mode for robots.
                """
                assert (
                    tm_robots in cls.registry_robots
                ), f"TaskMode '{tm_robots}' for robots is not registered!"
                self.__tm_robots = cls.registry_robots[tm_robots]()(props=self)
                self.__param_tm_robots = tm_robots

            def set_tm_obstacles(
                    self, tm_obstacles: Constants.TaskMode.TM_Obstacles):
                """
                Sets the task mode for obstacles.

                Args:
                    tm_obstacles (Constants.TaskMode.TM_Obstacles): The task mode for obstacles.
                """
                assert (
                    tm_obstacles in cls.registry_obstacles
                ), f"TaskMode '{tm_obstacles}' for obstacles is not registered!"
                self.__tm_obstacles = cls.registry_obstacles[tm_obstacles]()(
                    props=self)
                self.__param_tm_obstacles = tm_obstacles

            def _reset_task(self, **kwargs):
                """
                Reset the task by updating task modes, resetting modules, and spawning obstacles.

                Args:
                    **kwargs: Additional keyword arguments for resetting the task.

                Returns:
                    None
                """
                try:
                    self.__reset_start.publish(std_msgs.Empty())

                    if not self._train_mode:
                        if (
                            new_tm_robots := self.node.Configuration.TaskMode.TM_ROBOTS.value
                        ) != self.__param_tm_robots:
                            self.set_tm_robots(new_tm_robots)

                        if (
                            new_tm_obstacles := self.node.Configuration.TaskMode.TM_OBSTACLES.value
                        ) != self.__param_tm_obstacles:
                            self.set_tm_obstacles(new_tm_obstacles)

                    for module in self.__modules:
                        module.before_reset()

                    self.__tm_robots.reset(**kwargs)
                    obstacles, dynamic_obstacles = self.__tm_obstacles.reset(
                        **kwargs)

                    def respawn():
                        self.obstacle_manager.spawn_obstacles(obstacles)
                        self.obstacle_manager.spawn_dynamic_obstacles(
                            dynamic_obstacles)

                    self.obstacle_manager.respawn(respawn)

                    for module in self.__modules:
                        module.after_reset()

                    self.last_reset_time = self.clock.clock.sec

                except Exception as e:
                    self.node.get_logger().error(repr(e))
                    rclpy.shutdown()
                    raise Exception("reset error!") from e

                finally:
                    self.__reset_end.publish(std_msgs.Empty())

            def _mutex_reset_task(self, **kwargs):
                """
                Executes a reset task while ensuring mutual exclusion.

                This function acquires a mutex lock to ensure that only one reset task is executed at a time.
                It sets a parameter to indicate that the system is resetting, publishes a reset start message,
                performs the reset task, and then publishes a reset end message. If any exception occurs during
                the reset task, it logs the error, shuts down the ROS node, and raises an exception.

                Args:
                    kwargs: Additional keyword arguments.

                Raises:
                    Exception: If an error occurs during the reset task.

                """
                while self.__reset_mutex:
                    rclpy.sleep(0.001)
                self.__reset_mutex = True

                try:
                    rosparam_set(self.PARAM_RESETTING, True)
                    self._reset_task()

                except Exception as e:
                    raise e

                finally:
                    rosparam_set(self.PARAM_RESETTING, False)
                    self.__reset_mutex = False

            def reset(self, **kwargs):
                """
                Resets the task.

                Args:
                    **kwargs: Arbitrary keyword arguments.
                """
                self._force_reset = False
                if self._train_mode:
                    self._reset_task(**kwargs)
                else:
                    self._mutex_reset_task(**kwargs)

            @property
            def is_done(self) -> bool:
                """
                Checks if the task is done.

                Returns:
                    bool: True if the task is done, False otherwise.
                """
                return self._force_reset or self.__tm_robots.done

            def set_robot_position(self, position: PositionOrientation):
                """
                Sets the position of the robot.

                Args:
                    position (PositionOrientation): The position and orientation of the robot.
                """
                self.__tm_robots.set_position(position)

            def set_robot_goal(self, position: PositionOrientation):
                """
                Sets the goal position for the robot.

                Args:
                    position (PositionOrientation): The goal position for the robot.
                """
                self.__tm_robots.set_goal(position)

            def force_reset(self):
                self._force_reset = True

        return CombinedTask
