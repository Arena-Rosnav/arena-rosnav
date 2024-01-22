from typing import Any, Dict, List, Type

from task_generator.shared import Namespace

from .observation_units.base_collector_unit import BaseCollectorUnit
from .observation_units.collector_unit import CollectorUnit
from .observation_units.globalplan_collector_unit import GlobalplanCollectorUnit
from .observation_units.pedsim_collector_unit import PedsimStateCollectorUnit
from .observation_units.aggregate_collector_unit import AggregateCollectorUnit
from .observation_units.semantic_ped_unit import SemanticAggregateUnit


class ObservationManager:
    """
    Class to manage observation units and collect observations.
    Each unit is responsible for a dedicated observation type.

    Attributes:
        _ns (Namespace): The namespace object.
        _obs_structur (List[Type[CollectorUnit]]): The list of observation unit types.
        _observation_units (List[CollectorUnit]): The list of observation unit instances to retrieve information.
    """

    _ns: Namespace
    _obs_structur: List[Type[CollectorUnit]]
    _observation_units: List[CollectorUnit]

    def __init__(self, ns: Namespace, obs_structur: List[CollectorUnit] = None) -> None:
        """
        Initialize ObservationManager with namespace and observation structure.

        Args:
            ns (Namespace): The namespace object.
            obs_structur (List[CollectorUnit], optional): The list of observation unit types. Defaults to None.
        """
        self._ns = ns
        self._obs_structur = obs_structur or [
            BaseCollectorUnit,
            GlobalplanCollectorUnit,
            SemanticAggregateUnit,
        ]
        self._observation_units = self._instantiate_units()
        self._init_units()

    def _instantiate_units(self) -> List[CollectorUnit]:
        """
        Instantiate observation units based on the provided structure.

        Returns:
            List[CollectorUnit]: List of instantiated observation units.
        """
        return [
            collector_class(ns=self._ns, observation_manager=self)
            for collector_class in self._obs_structur
        ]

    def _init_units(self) -> None:
        """
        Initialize all observation units.
        """
        for unit in self._observation_units:
            unit.init_subs()

    def get_observations(self, *args, **kwargs) -> Dict[str, Any]:
        """
        Get observations from all observation units.

        Returns:
            Dict[str, Any]: Dictionary containing observations from all units.
        """
        obs_dict = {}
        for unit in self._observation_units:
            obs_dict = unit.get_observations(obs_dict=obs_dict, **kwargs)
        return obs_dict
