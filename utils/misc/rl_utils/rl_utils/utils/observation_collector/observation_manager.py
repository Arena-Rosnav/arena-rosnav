from typing import Any, Dict, List, Type

from task_generator.shared import Namespace

from .observation_units.base_collector_unit import BaseCollectorUnit
from .observation_units.collector_unit import CollectorUnit
from .observation_units.globalplan_collector_unit import GlobalplanCollectorUnit
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

    def __init__(
        self,
        ns: Namespace,
        obs_structur: List[CollectorUnit] = None,
        obs_unit_kwargs: dict = None,
    ) -> None:
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
        obs_unit_kwargs = obs_unit_kwargs or {}
        self._inititialize_units(**obs_unit_kwargs)

    def _inititialize_units(self, **kwargs) -> None:
        """
        Initialize all observation units.
        """
        self._observation_units = self._instantiate_units(**kwargs)

        for unit in self._observation_units:
            unit.init_subs()

    def _instantiate_units(self, **kwargs) -> List[CollectorUnit]:
        """
        Instantiate observation units based on the provided structure.

        Returns:
            List[CollectorUnit]: List of instantiated observation units.
        """
        return [
            collector_class(ns=self._ns, observation_manager=self, **kwargs)
            for collector_class in self._obs_structur
        ]

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
