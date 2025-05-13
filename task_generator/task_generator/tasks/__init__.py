import abc
import typing

import rclpy.publisher
import rosgraph_msgs.msg as rosgraph_msgs
from arena_rclpy_mixins.ROSParamServer import ROSParamServer
from arena_rclpy_mixins.shared import Namespace

from task_generator import NodeInterface
from task_generator.constants import Constants
from task_generator.manager.environment_manager import EnvironmentManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.manager.world_manager.world_manager_ros import WorldManager
from task_generator.shared import PositionOrientation
from task_generator.utils import ModelLoader


class Props_Manager:
    environment_manager: EnvironmentManager
    robot_managers: dict[str, RobotManager]
    world_manager: WorldManager


class Props_Modelloader:
    model_loader: ModelLoader
    dynamic_model_loader: ModelLoader


class Props_Namespace:
    namespace: str
    namespace_prefix: str


class Props_(Props_Manager, Props_Modelloader, Props_Namespace):
    clock: rosgraph_msgs.Clock


class Namespaced:
    _namespace: typing.ClassVar[Namespace] = Namespace('').ParamNamespace()

    @classmethod
    def namespace(cls, *path: str) -> Namespace:
        return cls._namespace(*path)


class TaskMode(NodeInterface, Namespaced):
    _PROPS: Props_

    def __init__(self, props: Props_, **kwargs):
        NodeInterface.__init__(self, **kwargs)
        self._PROPS = props


class Task(Props_, abc.ABC):
    """
    Base class for defining tasks in the task generator module.

    Attributes:
        last_reset_time (int): The time of the last reset.

    Constants:
        TOPIC_RESET_START (str): The topic for the reset start event.
        TOPIC_RESET_END (str): The topic for the reset end event.
        PARAM_RESETTING (str): The parameter for resetting.

    Private Attributes:
        __reset_start (rospy.Publisher): The publisher for the reset start event.
        __reset_end (rospy.Publisher): The publisher for the reset end event.
        __reset_mutex (bool): The mutex for resetting.

    Args:
        environment_manager (ObstacleManager): The obstacle manager.
        robot_managers (dict[str, RobotManager]): The dict of robot managers.
        world_manager (WorldManager): The world manager.
        namespace (str, optional): The namespace for the task. Defaults to "".

    Raises:
        NotImplementedError: This class is meant to be subclassed and not instantiated directly.

    Methods:
        reset(**kwargs): Reset the task.
        is_done() -> bool: Check if the task is done.
        robot_names() -> list[str]: Get the names of the robots in the task.
        _clock_callback(clock: rosgraph_msgs.Clock): Callback function for the clock message.
        set_robot_position(position: PositionOrientation): Set the position of the robot.
        set_robot_goal(position: PositionOrientation): Set the goal position of the robot.
    """

    last_reset_time: int

    TOPIC_RESET_START = "reset_start"
    TOPIC_RESET_END = "reset_end"
    PARAM_RESETTING = "resetting"

    @classmethod
    def declare_parameters(cls, node: ROSParamServer):
        node.ROSParam[bool](cls.PARAM_RESETTING, True)

    __reset_start: rclpy.publisher.Publisher
    __reset_end: rclpy.publisher.Publisher
    __reset_mutex: bool

    @abc.abstractmethod
    def __init__(
        self,
        *args,
        environment_manager: EnvironmentManager,
        robot_managers: dict[str, RobotManager],
        world_manager: WorldManager,
        namespace: str = "",
        **kwargs
    ):
        ...

    @abc.abstractmethod
    def reset(self, **kwargs) -> None:
        ...

    @property
    def is_done(self) -> bool:
        return False

    @property
    def robot_names(self) -> list[str]:
        return list(self.robot_managers.keys())

    def _clock_callback(self, clock: rosgraph_msgs.Clock):
        self.clock = clock

    @abc.abstractmethod
    def set_robot_position(self, position: PositionOrientation):
        ...

    @abc.abstractmethod
    def set_robot_goal(self, position: PositionOrientation):
        ...


from .task_factory import TaskFactory  # noqa


def declare_modules():
    @TaskFactory.register_module(Constants.TaskMode.TM_Module.BENCHMARK)
    def _benchmark():
        from .modules.benchmark import Mod_Benchmark
        return Mod_Benchmark

    @TaskFactory.register_module(
        Constants.TaskMode.TM_Module.CLEAR_FORBIDDEN_ZONES)
    def _clear_forbidden_zones():
        from .modules.clear_forbidden_zones import Mod_ClearForbiddenZones
        return Mod_ClearForbiddenZones

    @TaskFactory.register_module(Constants.TaskMode.TM_Module.DYNAMIC_MAP)
    def _dynamic_map():
        from .modules.dynamic_map import Mod_DynamicMap
        return Mod_DynamicMap

    @TaskFactory.register_module(Constants.TaskMode.TM_Module.RVIZ_UI)
    def _rviz_ui():
        from .modules.rviz_ui import Mod_OverrideRobot
        return Mod_OverrideRobot

    @TaskFactory.register_module(Constants.TaskMode.TM_Module.STAGED)
    def _staged():
        from .modules.staged import Mod_Staged
        return Mod_Staged


def declare_obstacles():
    @TaskFactory.register_obstacles(
        Constants.TaskMode.TM_Obstacles.PARAMETRIZED)
    def _parametrized():
        from .obstacles.parametrized import TM_Parametrized
        return TM_Parametrized

    @TaskFactory.register_obstacles(Constants.TaskMode.TM_Obstacles.RANDOM)
    def _random():
        from .obstacles.random import TM_Random
        return TM_Random

    @TaskFactory.register_obstacles(Constants.TaskMode.TM_Obstacles.SCENARIO)
    def _scenario():
        from .obstacles.scenario import TM_Scenario
        return TM_Scenario

    @TaskFactory.register_obstacles(Constants.TaskMode.TM_Obstacles.ENVIRONMENT)
    def _environment():
        from .obstacles.environment import TM_Environment
        return TM_Environment


def declare_robots():
    @TaskFactory.register_robots(Constants.TaskMode.TM_Robots.EXPLORE)
    def _explore():
        from .robots.explore import TM_Explore
        return TM_Explore

    @TaskFactory.register_robots(Constants.TaskMode.TM_Robots.GUIDED)
    def _guided():
        from .robots.guided import TM_Guided
        return TM_Guided

    @TaskFactory.register_robots(Constants.TaskMode.TM_Robots.RANDOM)
    def _random():
        from .robots.random import TM_Random
        return TM_Random

    @TaskFactory.register_robots(Constants.TaskMode.TM_Robots.SCENARIO)
    def _scenario():
        from .robots.scenario import TM_Scenario
        return TM_Scenario


declare_modules()
declare_obstacles()
declare_robots()
