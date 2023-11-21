from typing import List, Dict, Any

from task_generator.shared import Namespace
from .observation_units.collector_unit import CollectorUnit
from .observation_units.base_collector_unit import BaseCollectorUnit
from .observation_units.globalplan_collector_unit import GlobalplanCollectorUnit


class ObservationManager:
    def __init__(self, ns: Namespace, obs_struct: List[CollectorUnit] = None) -> None:
        self._ns = ns
        self._obs_structur = [BaseCollectorUnit, GlobalplanCollectorUnit] or obs_struct
        self._observation_units = self._instantiate_units()
        self._init_units()

        self._empty_dict = {}

    def _instantiate_units(self) -> List[CollectorUnit]:
        return [
            collector_class(ns=self._ns, observation_manager=self)
            for collector_class in self._obs_structur
        ]

    def _init_units(self):
        for unit in self._observation_units:
            unit.init_subs()

    def get_observations(self, *args, **kwargs) -> Dict[str, Any]:
        obs_dict = {}
        for unit in self._observation_units:
            obs_dict = unit.get_observations(obs_dict=obs_dict, **kwargs)
        return obs_dict
