import dataclasses
import json
import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from task_generator.constants import Constants
from task_generator.shared import DynamicObstacle, Obstacle, rosparam_get
from task_generator.tasks.obstacles import Obstacles, TM_Obstacles

import rclpy
from rcl_interfaces.msg import SetParametersResult


@dataclasses.dataclass
class _Config:
    static: List[Obstacle]
    dynamic: List[DynamicObstacle]


class TM_Scenario(TM_Obstacles):

    _config: _Config

    @classmethod
    def prefix(cls, *args):
        return super().prefix("scenario", *args)

    def __init__(self, **kwargs):
        TM_Obstacles.__init__(self, **kwargs)

        self.node.declare_parameter('SCENARIO_file', '')
        self.node.add_on_set_parameters_callback(self.parameters_callback)

        # Initial configuration
        self.reconfigure(
            {'SCENARIO_file': self.node.get_parameter('SCENARIO_file').value})

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'SCENARIO_file':
                self.reconfigure({'SCENARIO_file': param.value})
        return SetParametersResult(successful=True)

    def reconfigure(self, config):
        scenario_file = config['SCENARIO_file']

        package_share_directory = get_package_share_directory(
            'simulation-setup')
        map_file = self.node.get_parameter('map_file').value

        scenario_path = os.path.join(
            package_share_directory,
            "worlds",
            map_file,
            "scenarios",
            scenario_file
        )

        with open(scenario_path) as f:
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
                        obs.get("model", Constants.DEFAULT_PEDESTRIAN_MODEL))
                )
                for obs in scenario.get("obstacles", {}).get("dynamic", [])
            ]
        )

    def reset(self, **kwargs) -> Obstacles:
        return self._config.static, self._config.dynamic
