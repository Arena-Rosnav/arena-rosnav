from dataclasses import dataclass
import os
from typing import List, Optional, TypeVar
from task_generator.constants import Constants
from task_generator.manager.map_manager import MapManager
from task_generator.manager.robot_manager import RobotManager

import rospy

from task_generator.tasks.base_task import BaseTask
from task_generator.tasks.task_factory import TaskFactory
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.tasks.utils import RandomInterface, Scenario, ScenarioInterface
from task_generator.utils import rosparam_get





@TaskFactory.register(Constants.TaskMode.SCENARIO)
class ScenarioTask(BaseTask, ScenarioInterface):
    """
    Use Scenario
    """

    def __init__(
        self,
        obstacle_manager: ObstacleManager,
        robot_managers: List[RobotManager],
        map_manager: MapManager,
        **kwargs
    ):
        BaseTask.__init__(
            self,
            obstacle_manager=obstacle_manager,
            robot_managers=robot_managers,
            map_manager=map_manager,
            **kwargs
        )
        ScenarioInterface.__init__(self)

        scenario_file_name: Optional[str] = rosparam_get(str, "~configuration/task_mode/scenario/scenario_file", "") or None

        if scenario_file_name is None:
            scenario = kwargs.get("scenario")
            assert isinstance(scenario, Scenario), "couldn't get scenario file"
            self._scenario = scenario
        else:
            with open(os.path.join(ScenarioInterface.SCENARIO_BASE_PATH, scenario_file_name), "r") as file:
                self._scenario = self._read_scenario_file(file.read())

        self._check_scenario(self._scenario)

    @BaseTask.reset_helper(parent=BaseTask)
    def reset(self, **kwargs):

        def callback():
            self._obstacle_manager.respawn(lambda: ScenarioInterface._setup_scenario(self, self._scenario))
            return False
    
        return callback

    
    