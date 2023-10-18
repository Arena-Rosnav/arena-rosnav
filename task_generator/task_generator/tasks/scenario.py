from dataclasses import dataclass
from typing import List, Optional, TypeVar
from task_generator.constants import Constants
from task_generator.manager.map_manager import MapManager
from task_generator.manager.robot_manager import RobotManager

import rospy

from task_generator.tasks.base_task import BaseTask
from task_generator.tasks.task_factory import TaskFactory
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.tasks.utils import Scenario, ScenarioInterface





@TaskFactory.register(Constants.TaskMode.SCENARIO)
class ScenarioTask(BaseTask, ScenarioInterface):
    """
    Load Scenario
    """

    _reset_count: int
    _desired_resets: int

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

        scenario_file_path: Optional[str] = str(rospy.get_param("~scenario_json_path", "")) or None

        if scenario_file_path is None:
            scenario = kwargs.get("scenario")
            assert isinstance(scenario, Scenario), "couldn't get scenario file"
            self._scenario = scenario
        else:
            with open(scenario_file_path, "r") as file:
                self._scenario = self._read_scenario_file(file.read())

        self._check_scenario(self._scenario)

        self._reset_count = 0

        self._desired_resets = self._scenario.resets
        if not self._desired_resets > 0:
            rospy.loginfo(
                f"Setting resets to default of {Constants.Scenario.RESETS_DEFAULT}"
            )
            self._desired_resets = Constants.Scenario.RESETS_DEFAULT

    @BaseTask.reset_helper(parent=BaseTask)
    def reset(self, **kwargs):

        def callback():
            if self._reset_count >= self._desired_resets:
                return True
            self._obstacle_manager.respawn(lambda: ScenarioInterface._setup_scenario(self, self._scenario))
            
            self._reset_count += 1
            return False
    
        return callback

    
    