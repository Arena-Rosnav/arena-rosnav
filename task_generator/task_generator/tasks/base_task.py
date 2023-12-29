import os
from typing import Any, Callable, Dict, List, Optional, Tuple, Type

from rospkg import RosPack
import rospy

import rosgraph_msgs.msg as rosgraph_msgs
from task_generator.shared import Namespace
import std_msgs.msg as std_msgs

from task_generator.constants import Constants
from task_generator.manager.world_manager import WorldManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.utils import ModelLoader


class Props_Manager:
    obstacle_manager: ObstacleManager
    robot_managers: List[RobotManager]
    world_manager: WorldManager


class Props_Modelloader:
    model_loader: ModelLoader
    dynamic_model_loader: ModelLoader


class Props_Namespace:
    namespace: str
    namespace_prefix: str


class Props_(Props_Manager, Props_Modelloader, Props_Namespace):
    ...


class BaseTask(Props_):
    """
    Base Task as parent class for all other tasks.
    """

    clock: rosgraph_msgs.Clock
    last_reset_time: int

    TOPIC_RESET_START = "reset_start"
    TOPIC_RESET_END = "reset_end"
    PARAM_RESETTING = "resetting"

    __reset_start: rospy.Publisher
    __reset_end: rospy.Publisher

    def __init__(
        self,
        obstacle_manager: ObstacleManager,
        robot_managers: List[RobotManager],
        world_manager: WorldManager,
        namespace: str = "",
        *args,
        **kwargs
    ):
        self.namespace = Namespace(namespace)
        self.namespace_prefix = self.namespace.simulation_ns

        self.obstacle_manager = obstacle_manager
        self.robot_managers = robot_managers
        self.world_manager = world_manager

        self.__reset_start = rospy.Publisher(
            self.TOPIC_RESET_START, std_msgs.Empty, queue_size=1
        )
        self.__reset_end = rospy.Publisher(
            self.TOPIC_RESET_END, std_msgs.Empty, queue_size=1
        )

        rospy.Subscriber("/clock", rosgraph_msgs.Clock, self._clock_callback)
        self.last_reset_time = 0
        self.clock = rosgraph_msgs.Clock()

        self.set_up_robot_managers()

        self.model_loader = ModelLoader(
            os.path.join(
                RosPack().get_path("arena-simulation-setup"),
                "obstacles",
                "static_obstacles",
            )
        )
        self.dynamic_model_loader = ModelLoader(
            os.path.join(
                RosPack().get_path("arena-simulation-setup"),
                "obstacles",
                "dynamic_obstacles",
            )
        )

    @staticmethod
    def reset_helper(parent: Type["BaseTask"]) -> Callable[..., Callable[..., bool]]:
        """
        Decorate reset(self, ...)->Callable[..., bool] in any BaseTask subclass with @BaseTask.reset_helper(parent=parent_class) to bind into the reset chain.
        First the reset body is called, then the chain is traversed up to BaseTask and then back down to the callback returned by reset. Return True from this callback to indicate all tasks are completed and the simulation can be shut down.
        @parent: direct parent class
        """

        def outer(
            fn: Callable[..., Tuple[Dict[str, Any], Optional[Callable[[], bool]]]]
        ) -> Callable[..., bool]:
            def _reset(self, callback: Callable[[], bool], **kwargs) -> bool:
                overrides, fn_callback = fn(self, **kwargs)
                if fn_callback is None:
                    return False
                return (
                    parent.reset(self, callback=callback, **{**overrides, **kwargs})
                    or fn_callback()
                )

            return _reset

        return outer

    _reset_semaphore: bool = False

    def reset(self, callback: Callable[[], bool], **kwargs) -> bool:
        """
        Calls a passed reset function (usually the tasks own reset)
        inside a loop so when the callback fails once it is tried
        again. After MAX_RESET_FAIL_TIMES the reset is considered
        as fail and the simulation is shut down.
        """

        return_val = False

        if self._reset_semaphore:
            return False
        self._reset_semaphore = True

        try:
            rospy.set_param(self.PARAM_RESETTING, True)
            self.__reset_start.publish()
            return_val = callback()
            rospy.set_param(self.PARAM_RESETTING, False)
            self.__reset_end.publish()
            self.last_reset_time = self.clock.clock.secs

        except rospy.ServiceException as e:
            rospy.logerr(repr(e))
            rospy.signal_shutdown("Reset error!")
            raise Exception("reset error!")

        finally:
            self._reset_semaphore = False

        return return_val

    @property
    def is_done(self) -> bool:
        if self.clock.clock.secs - self.last_reset_time > Constants.TIMEOUT:
            return True

        for manager in self.robot_managers:
            if not manager.is_done:
                return False

        return True

    @property
    def robot_names(self) -> List[str]:
        return [manager.name for manager in self.robot_managers]

    def set_up_robot_managers(self):
        for manager in self.robot_managers:
            manager.set_up_robot()

    def _clock_callback(self, clock: rosgraph_msgs.Clock):
        self.clock = clock
