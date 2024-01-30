import dataclasses
import json
import os
from typing import List

import rospkg
import rospy
from task_generator.constants import Constants
from task_generator.shared import DynamicObstacle, Obstacle, rosparam_get
from task_generator.tasks.obstacles import Obstacles, TM_Obstacles
from task_generator.tasks.task_factory import TaskFactory

import dynamic_reconfigure.client


@dataclasses.dataclass
class _Config:
    static: List[Obstacle]
    dynamic: List[DynamicObstacle]


@TaskFactory.register_obstacles(Constants.TaskMode.TM_Obstacles.SCENARIO)
class TM_Scenario(TM_Obstacles):

    _config: _Config

    @classmethod
    def prefix(cls, *args):
        return super().prefix("scenario", *args)
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        dynamic_reconfigure.client.Client(
            name=self.NODE_CONFIGURATION,
            config_callback=self.reconfigure
        )

    def reconfigure(self, config):

        rospy.logdebug(f"RECONFIGURED SCENARIO TO FILE {rosparam_get(str, self.NODE_CONFIGURATION('SCENARIO_file'))}")

        with open(
            os.path.join(
                rospkg.RosPack().get_path("arena_bringup"),
                "configs",
                "scenarios",
                rosparam_get(str, self.NODE_CONFIGURATION("SCENARIO_file"))
            )
        ) as f:
            scenario = json.load(f)

        self._config = _Config(
            static=[
                Obstacle.parse(
                    obs,
                    model=self._PROPS.model_loader.bind(obs["model"])
                )
                for obs in scenario.get("obstacles", {}).get("static", []) + scenario.get("obstacles", {}).get("interactive", [])
            ],
            dynamic=[
                DynamicObstacle.parse(
                    obs,
                    model=self._PROPS.dynamic_model_loader.bind(
                        obs["model"])
                )
                for obs in scenario.get("obstacles", {}).get("dynamic", [])
            ]
        )

    def reset(self, **kwargs) -> Obstacles:

        return self._config.static, self._config.dynamic
