import os
from typing import Dict, List, Type

import rospy
from rospkg import RosPack

from task_generator.constants import Constants
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.manager.world_manager import WorldManager
from task_generator.shared import PositionOrientation, rosparam_get
from task_generator.tasks import Task
from task_generator.tasks.modules import TM_Module
from task_generator.tasks.obstacles import TM_Obstacles
from task_generator.tasks.robots import TM_Robots

import std_msgs.msg as std_msgs
import rosgraph_msgs.msg as rosgraph_msgs
import training.srv as training_srvs

from task_generator.utils import ModelLoader


class TaskFactory:
    registry_obstacles: Dict[Constants.TaskMode.TM_Obstacles, Type[TM_Obstacles]] = {}
    registry_robots: Dict[Constants.TaskMode.TM_Robots, Type[TM_Robots]] = {}
    registry_module: Dict[Constants.TaskMode.TM_Module, Type[TM_Module]] = {}

    @classmethod
    def register_obstacles(cls, name: Constants.TaskMode.TM_Obstacles):
        def inner_wrapper(wrapped_class: Type[TM_Obstacles]):
            assert (
                name not in cls.registry_obstacles
            ), f"TaskMode '{name}' for obstacles already exists!"
            assert issubclass(wrapped_class, TM_Obstacles)

            cls.registry_obstacles[name] = wrapped_class
            return wrapped_class

        return inner_wrapper

    @classmethod
    def register_robots(cls, name: Constants.TaskMode.TM_Robots):
        def inner_wrapper(wrapped_class: Type[TM_Robots]):
            assert (
                name not in cls.registry_obstacles
            ), f"TaskMode '{name}' for robots already exists!"
            assert issubclass(wrapped_class, TM_Robots)

            cls.registry_robots[name] = wrapped_class
            return wrapped_class

        return inner_wrapper

    @classmethod
    def register_module(cls, name: Constants.TaskMode.TM_Module):
        def inner_wrapper(wrapped_class: Type[TM_Module]):
            assert (
                name not in cls.registry_obstacles
            ), f"TaskMode '{name}' for module already exists!"
            assert issubclass(wrapped_class, TM_Module)

            cls.registry_module[name] = wrapped_class
            return wrapped_class

        return inner_wrapper

    @classmethod
    def combine(cls, modules: List[Constants.TaskMode.TM_Module] = []) -> Type[Task]:
        for module in modules:
            assert (
                module in cls.registry_module
            ), f"Module '{module}' is not registered!"

        class CombinedTask(Task):
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
                robot_managers: List[RobotManager],
                world_manager: WorldManager,
                namespace: str = "",
                *args,
                **kwargs,
            ):
                """
                Initializes a CombinedTask object.

                Args:
                    obstacle_manager (ObstacleManager): The obstacle manager for the task.
                    robot_managers (List[RobotManager]): The list of robot managers for the task.
                    world_manager (WorldManager): The world manager for the task.
                    namespace (str, optional): The namespace for the task. Defaults to "".
                    *args: Variable length argument list.
                    **kwargs: Arbitrary keyword arguments.
                """
                self._force_reset = False
                self.namespace = namespace

                self.obstacle_manager = obstacle_manager
                self.robot_managers = robot_managers
                self.world_manager = world_manager

                self._train_mode = rosparam_get(bool, "/train_mode", False)

                self.__reset_start = rospy.Publisher(
                    self.TOPIC_RESET_START, std_msgs.Empty, queue_size=1
                )
                self.__reset_end = rospy.Publisher(
                    self.TOPIC_RESET_END, std_msgs.Empty, queue_size=1
                )
                self.__reset_mutex = False

                rospy.Subscriber("/clock", rosgraph_msgs.Clock, self._clock_callback)
                self.last_reset_time = 0
                self.clock = rosgraph_msgs.Clock()

                self.set_up_robot_managers()

                self.model_loader = ModelLoader(
                    os.path.join(
                        RosPack().get_path("arena_simulation_setup"),
                        "entities",
                        "obstacles",
                        "static",
                    )
                )
                self.dynamic_model_loader = ModelLoader(
                    os.path.join(
                        RosPack().get_path("arena_simulation_setup"),
                        "entities",
                        "obstacles",
                        "dynamic",
                    )
                )

                self.__param_tm_obstacles = None
                self.__param_tm_robots = None
                self.__modules = [
                    cls.registry_module[module](task=self) for module in modules
                ]

                if self._train_mode:
                    self.set_tm_robots(Constants.TaskMode.TM_Robots(rospy.get_param("tm_robots")))
                    self.set_tm_obstacles(Constants.TaskMode.TM_Obstacles(rospy.get_param("tm_obstacles")))

            def set_tm_robots(self, tm_robots: Constants.TaskMode.TM_Robots):
                """
                Sets the task mode for robots.

                Args:
                    tm_robots (Constants.TaskMode.TM_Robots): The task mode for robots.
                """
                assert (
                    tm_robots in cls.registry_robots
                ), f"TaskMode '{tm_robots}' for robots is not registered!"
                self.__tm_robots = cls.registry_robots[tm_robots](props=self)
                self.__param_tm_robots = tm_robots

            def set_tm_obstacles(self, tm_obstacles: Constants.TaskMode.TM_Obstacles):
                """
                Sets the task mode for obstacles.

                Args:
                    tm_obstacles (Constants.TaskMode.TM_Obstacles): The task mode for obstacles.
                """
                assert (
                    tm_obstacles in cls.registry_obstacles
                ), f"TaskMode '{tm_obstacles}' for obstacles is not registered!"
                self.__tm_obstacles = cls.registry_obstacles[tm_obstacles](props=self)
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
                    self.__reset_start.publish()

                    if not self._train_mode:
                        if (
                            new_tm_robots := Constants.TaskMode.TM_Robots(
                                rosparam_get(str, self.PARAM_TM_ROBOTS)
                            )
                        ) != self.__param_tm_robots:
                            self.set_tm_robots(new_tm_robots)

                        if (
                            new_tm_obstacles := Constants.TaskMode.TM_Obstacles(
                                rosparam_get(str, self.PARAM_TM_OBSTACLES)
                            )
                        ) != self.__param_tm_obstacles:
                            self.set_tm_obstacles(new_tm_obstacles)

                    for module in self.__modules:
                        module.before_reset()

                    self.__tm_robots.reset(**kwargs)
                    obstacles, dynamic_obstacles = self.__tm_obstacles.reset(**kwargs)

                    def respawn():
                        self.obstacle_manager.spawn_obstacles(obstacles)
                        self.obstacle_manager.spawn_dynamic_obstacles(dynamic_obstacles)

                    self.obstacle_manager.respawn(respawn)

                    for module in self.__modules:
                        module.after_reset()

                    self.last_reset_time = self.clock.clock.secs

                except rospy.ServiceException as e:
                    rospy.logerr(repr(e))
                    rospy.signal_shutdown("Reset error!")
                    raise Exception("reset error!") from e

                finally:
                    self.__reset_end.publish()

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
                    rospy.sleep(0.001)
                self.__reset_mutex = True

                try:
                    rospy.set_param(self.PARAM_RESETTING, True)
                    self._reset_task()

                except Exception as e:
                    raise e

                finally:
                    rospy.set_param(self.PARAM_RESETTING, False)
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


from .obstacles.random import TM_Random
from .obstacles.scenario import TM_Scenario
from .obstacles.parametrized import TM_Parametrized

from .robots.random import TM_Random
from .robots.guided import TM_Guided
from .robots.explore import TM_Explore
from .robots.scenario import TM_Scenario

from .modules.clear_forbidden_zones import Mod_ClearForbiddenZones
from .modules.dynamic_map import Mod_DynamicMap
from .modules.rviz_ui import Mod_OverrideRobot
from .modules.staged import Mod_Staged
from .modules.benchmark import Mod_Benchmark
