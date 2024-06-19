import functools
from typing import Dict, List

import rospy
from task_generator.shared import Namespace
from .traversal import explore_hierarchy

from . import (
    GenericObservation,
    ObservationCollectorUnit,
    ObservationDict,
    ObservationGeneratorUnit,
    ObservationGeneric,
    TypeObservationGeneric,
)

from .utils.topic import get_topic


class ObservationManager:
    """
    Class to manage observation units and collect observations.
    Each unit is responsible for a dedicated observation type.

    Attributes:
        _ns (Namespace): The namespace object.
        _obs_structur (List[Type[ObservationCollectorUnit]]): The list of observation unit types.
        _collectors (List[ObservationCollectorUnit]): The list of observation unit instances to retrieve information.
        _generators (List[ObservationGeneratorUnit]): The list of observation generator unit instances to generate observations.
        _collectable_observations (Dict[str, GenericObservation]): A dictionary to store the collectable observations.
        _subscribers (Dict[str, rospy.Subscriber]): A dictionary to store the subscribers for each observation collector.
    """

    _ns: Namespace
    _obs_structur: List[TypeObservationGeneric]
    _collectors: Dict[str, ObservationCollectorUnit]
    _generators: Dict[str, ObservationGeneratorUnit]
    _collectable_observations: Dict[str, GenericObservation]
    _subscribers: Dict[str, rospy.Subscriber]

    def __init__(
        self,
        ns: Namespace,
        obs_structur: List[TypeObservationGeneric],
        obs_unit_kwargs: dict = None,
        wait_for_obs: bool = True,
    ) -> None:
        """
        Initialize ObservationManager with namespace and observation structure.

        Args:
            ns (Namespace): The namespace object.
            obs_structur (List[ObservationCollectorUnit], optional): The list of observation unit types. Defaults to None.
            obs_unit_kwargs (dict, optional): Additional keyword arguments for observation units. Defaults to None.
        """
        self._ns = ns
        self._obs_structur = list(explore_hierarchy(obs_structur).keys())

        obs_unit_kwargs = obs_unit_kwargs or {}
        self._collectable_observations = {}
        self._subscribers = {}

        self._inititialize_units(obs_unit_kwargs=obs_unit_kwargs)
        self._init_units()

        self._wait_for_obs = wait_for_obs

    def _inititialize_units(self, obs_unit_kwargs: dict) -> None:
        """
        Initialize all observation units.
        """
        _collector_cls = [
            unit
            for unit in self._obs_structur
            if issubclass(unit, ObservationCollectorUnit)
        ]
        _generator_cls = [
            unit
            for unit in self._obs_structur
            if issubclass(unit, ObservationGeneratorUnit)
        ]

        self._collectors = {
            collector_class.name: collector_class(**obs_unit_kwargs)
            for collector_class in _collector_cls
        }
        self._generators = {
            generator_class.name: generator_class(**obs_unit_kwargs)
            for generator_class in _generator_cls
        }

    def _init_units(self) -> None:
        """
        Initialize observation units and subscribers.
        """
        for collector in self._collectors.values():
            observation_container = GenericObservation(
                initial_msg=collector.msg_data_class(),
                process_fnc=collector.preprocess,
            )

            self._collectable_observations[collector.name] = observation_container

            self._subscribers[collector.name] = rospy.Subscriber(
                get_topic(self._ns, collector.topic, collector.is_topic_agent_specific),
                collector.msg_data_class,
                functools.partial(observation_container.update),
            )

    def _invalidate_observations(self) -> None:
        """
        Invalidate all observations.
        """
        for collector in self._collectable_observations.values():
            collector.invalidate()

    def _wait_for_observation(self, name: str):
        rospy.wait_for_message(
            get_topic(
                self._ns,
                self._collectors[name].topic,
                self._collectors[name].is_topic_agent_specific,
            ),
            self._collectors[name].msg_data_class,
            timeout=5,
        )

    def _get_collectable_observations(
        self, obs_dict: ObservationDict
    ) -> ObservationDict:
        # Retrieve all observations from the collectors
        for name, observation in self._collectable_observations.items():
            if observation.stale:
                rospy.logdebug_throttle(1, f"Observation '{name}' IS STALE.")
                if self._wait_for_obs:
                    self._wait_for_observation(name)
            obs_dict[name] = observation.value

        self._invalidate_observations()
        return obs_dict

    def _get_generatable_observations(
        self, obs_dict: ObservationDict
    ) -> ObservationDict:
        # Generate observations from the generators
        for generator in self._generators.values():
            try:
                obs_dict[generator.name] = generator.generate(obs_dict=obs_dict)
            except KeyError as e:
                rospy.logwarn_once(
                    f"{e} \n Could not generate observation for '{generator.name}'."
                )
                obs_dict[generator.name] = None

        return obs_dict

    def get_observations(self, *args, **kwargs) -> ObservationDict:
        """
        Get observations from all observation units.

        Returns:
            ObservationDict: Dictionary containing observations from all units.
        """
        obs_dict = {}

        self._get_collectable_observations(obs_dict)
        self._get_generatable_observations(obs_dict)

        return obs_dict
