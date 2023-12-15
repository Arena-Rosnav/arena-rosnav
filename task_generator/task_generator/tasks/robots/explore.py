import math
import random
from typing import Dict
import genpy
import rospy
from task_generator.constants import Constants
from task_generator.shared import PositionOrientation, rosparam_get
from task_generator.tasks.robots.random import TM_Random
from task_generator.tasks.task_factory import TaskFactory


@TaskFactory.register_robots(Constants.TaskMode.TM_Robots.EXPLORE)
class TM_Explore(TM_Random):

    _timeouts: Dict[int, genpy.Time]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        self._timeouts = dict()
        for i in range(len(self._PROPS.robot_managers)):
            self._reset_timeout(i)

    @property
    def done(self) -> bool:
        for i, robot in enumerate(self._PROPS.robot_managers):
            if robot.is_done:
                waypoint = self._PROPS.world_manager.get_position_on_map(safe_dist=robot._robot_radius, forbid=False)
                self._set_goal(i, PositionOrientation(*waypoint, random.random()*2*math.pi))
            
            elif (self._PROPS.clock.clock - self._timeouts[i]).secs > rosparam_get(float, "TIMEOUT", 90):
                waypoint = self._PROPS.world_manager.get_position_on_map(safe_dist=robot._robot_radius, forbid=False)
                self._set_position(i, PositionOrientation(*waypoint, random.random()*2*math.pi))

        return False
    
    def _reset_timeout(self, index: int):
        self._timeouts[index] = self._PROPS.clock.clock

    def _set_position(self, index: int, position: PositionOrientation):
        self._reset_timeout(index)
        self._PROPS.robot_managers[index].reset(position, None)

    def _set_goal(self, index: int, position: PositionOrientation):
        self._reset_timeout(index)
        self._PROPS.robot_managers[index].reset(None, position)