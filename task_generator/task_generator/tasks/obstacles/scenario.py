from arena_rclpy_mixins.ROSParamServer import ROSParamT
from arena_simulation_setup.world import Scenario, World
from task_generator.tasks.obstacles import Obstacles, TM_Obstacles


class TM_Scenario(TM_Obstacles):

    _config: ROSParamT[Scenario]

    def _parse_scenario(self, scenario: str) -> Scenario:
        return World(self.node._world_manager.world_name).scenario(scenario).load()

    def reset(self, **kwargs) -> Obstacles:
        return self._config.value.static, self._config.value.dynamic

    def __init__(self, **kwargs):
        TM_Obstacles.__init__(self, **kwargs)
        self._config = self.node.ROSParam[Scenario](
            self.namespace('file'),
            'default.json',
            parse=self._parse_scenario,
        )
