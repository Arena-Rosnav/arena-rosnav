from typing import List
from task_generator.constants import Constants
from task_generator.shared import DynamicObstacle, Obstacle, rosparam_get
from task_generator.tasks.obstacles import Obstacles, TM_Obstacles
from task_generator.tasks.task_factory import TaskFactory

@TaskFactory.register_obstacles(Constants.TaskMode.TM_Obstacles.SCENARIO)
class TM_Scenario(TM_Obstacles):
    def reset(self, **kwargs) -> Obstacles:

        SCENARIO_STATIC_OBSTACLES: List[Obstacle] = [
            Obstacle.parse(
                obs,
                model=self._PROPS.model_loader.bind(obs["model"])
            )
            for obs in rosparam_get(list, "scenario/obstacles/static", [])
        ]

        SCENARIO_INTERACTIVE_OBSTACLES: List[Obstacle] = [
            Obstacle.parse(
                obs,
                model=self._PROPS.model_loader.bind(obs["model"])
            )
            for obs in rosparam_get(list, "scenario/obstacles/interactive", [])
        ]

        SCENARIO_DYNAMIC_OBSTACLES: List[DynamicObstacle] = [
            DynamicObstacle.parse(
                obs,
                model=self._PROPS.dynamic_model_loader.bind(obs["model"])
            )
            for obs in rosparam_get(list, "scenario/obstacles/interactive", [])
        ] 

        return SCENARIO_STATIC_OBSTACLES + SCENARIO_INTERACTIVE_OBSTACLES, SCENARIO_DYNAMIC_OBSTACLES