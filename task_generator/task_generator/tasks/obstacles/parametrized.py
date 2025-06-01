import os

from arena_rclpy_mixins.ROSParamServer import ROSParamT

from task_generator.shared import DynamicObstacle, Obstacle
from task_generator.tasks.obstacles import TM_Obstacles
from task_generator.tasks.obstacles.utils import ITF_Obstacle

from arena_simulation_setup.configs.parametrized import ParametrizedConfig, Parametrized


class TM_Parametrized(TM_Obstacles):

    _config: ROSParamT[ParametrizedConfig]

    def _parse(self, config_name: str) -> ParametrizedConfig:
        return Parametrized(config_name).load()

    def reset(self, **kwargs):
        dynamic_obstacles: list[DynamicObstacle] = []
        obstacles: list[Obstacle] = []

        # Create static obstacles
        for config in self._config.value.STATIC:
            for i in range(
                self.node.conf.General.RNG.value.integers(
                    config.min,
                    config.max,
                    endpoint=True
                )
            ):
                obstacle = ITF_Obstacle.create_obstacle(
                    self.node,
                    self._PROPS,
                    name=f'S_{config.model}_{i + 1}',
                    model=config.model
                )
                obstacle.extra["type"] = config.type
                obstacles.append(obstacle)

        # Create interactive obstacles
        for config in self._config.value.INTERACTIVE:
            for i in range(
                self.node.conf.General.RNG.value.integers(
                    config.min,
                    config.max,
                    endpoint=True
                )
            ):
                obstacle = ITF_Obstacle.create_obstacle(
                    self.node,
                    self._PROPS,
                    name=f'S_{config.model}_{i + 1}',
                    model=config.model
                )
                obstacle.extra["type"] = config.type
                obstacles.append(obstacle)

        # Create dynamic obstacles
        for config in self._config.value.DYNAMIC:
            for i in range(
                self.node.conf.General.RNG.value.integers(
                    config.min,
                    config.max,
                    endpoint=True
                )
            ):
                obstacle = ITF_Obstacle.create_dynamic_obstacle(
                    self.node,
                    self._PROPS,
                    name=f'S_{config.model}_{i + 1}',
                    model=config.model
                )
                obstacle.extra["type"] = config.type
                dynamic_obstacles.append(obstacle)

        return obstacles, dynamic_obstacles

    def __init__(self, **kwargs):
        TM_Obstacles.__init__(self, **kwargs)

        self._config = self.node.ROSParam[ParametrizedConfig](
            self.namespace('file'),
            '',
            parse=self._parse
        )
