
from typing import List
import rospy

from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.manager.world_manager import WorldManager
from task_generator.shared import PositionOrientation

from task_generator.utils import ModelLoader

import rosgraph_msgs.msg as rosgraph_msgs

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
    clock: rosgraph_msgs.Clock

class TaskMode:

    _PROPS: Props_

    def __init__(self, props: Props_, **kwargs):
        self._PROPS = props

class Task(Props_):
    last_reset_time: int

    TOPIC_RESET_START = "reset_start"
    TOPIC_RESET_END = "reset_end"
    PARAM_RESETTING = "resetting"

    __reset_start: rospy.Publisher
    __reset_end: rospy.Publisher
    __reset_mutex: bool

    def __init__(self,
        obstacle_manager: ObstacleManager,
        robot_managers: List[RobotManager],
        world_manager: WorldManager,
        namespace: str = "",
        *args, **kwargs
    ):
        raise NotImplementedError()

    def reset(self, **kwargs):
        ...

    @property
    def is_done(self) -> bool:
        return False

    @property
    def robot_names(self) -> List[str]:
        return [manager.name for manager in self.robot_managers]

    def set_up_robot_managers(self):
        for manager in self.robot_managers:
            manager.set_up_robot()

    def _clock_callback(self, clock: rosgraph_msgs.Clock):
        self.clock = clock

    def set_robot_position(self, position: PositionOrientation):
        ...

    def set_robot_goal(self, position: PositionOrientation):
        ...


import task_generator.tasks.modules # noqa
import task_generator.tasks.robots # noqa
import task_generator.tasks.obstacles # noqa