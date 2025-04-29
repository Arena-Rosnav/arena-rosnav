import json
import os
from typing import List

import attrs

from task_generator.constants import Constants
from task_generator.shared import DynamicObstacle, Obstacle
from task_generator.tasks.obstacles import Obstacles, TM_Obstacles
from task_generator.utils.ros_params import ROSParamT


@attrs.define()
class _ParsedConfig:
    static: List[Obstacle]
    dynamic: List[DynamicObstacle]


class TM_Scenario(TM_Obstacles):

    _config: ROSParamT[_ParsedConfig]

    def _parse_scenario(self, scenario_file: str) -> _ParsedConfig:

        scenario_path = os.path.join(
            self.node.conf.Arena.get_world_path(),
            "scenarios",
            scenario_file
        )

        with open(scenario_path) as f:
            scenario = json.load(f)

        return _ParsedConfig(
            static=[
                Obstacle.parse(
                    obs,
                    model=self._PROPS.model_loader.bind(obs["model"])
                )
                for obs
                in
                scenario.get("obstacles", {}).get("static", []) +
                scenario.get("obstacles", {}).get("interactive", [])
            ],
            dynamic=[
                DynamicObstacle.parse(
                    obs,
                    model=self._PROPS.dynamic_model_loader.bind(
                        obs.get("model", Constants.DEFAULT_PEDESTRIAN_MODEL))
                )
                for obs
                in
                scenario.get("obstacles", {}).get("dynamic", [])
            ]
        )

    def reset(self, **kwargs) -> Obstacles:
        return self._config.value.static, self._config.value.dynamic

    def __init__(self, **kwargs):
        TM_Obstacles.__init__(self, **kwargs)
        self._config = self.node.ROSParam[_ParsedConfig](
            self.namespace('file'),
            'default.json',
            parse=self._parse_scenario,
        )
