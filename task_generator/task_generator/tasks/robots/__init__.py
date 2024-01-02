from typing import List
from task_generator.constants import Constants, TaskConfig
from task_generator.shared import Obstacle, PositionOrientation
from task_generator.tasks import Props_, TaskMode

class TM_Robots(TaskMode):

    def __init__(self, **kwargs):
        TaskMode.__init__(self, **kwargs)

    def reset(self, **kwargs):
        ...
    
    def set_position(self, position: PositionOrientation):
        for robot in self._PROPS.robot_managers:
            robot.reset(position, None)

    def set_goal(self, position: PositionOrientation):
        for robot in self._PROPS.robot_managers:
            robot.reset(None, position)

    @property
    def done(self):
        return all(robot.is_done for robot in self._PROPS.robot_managers)