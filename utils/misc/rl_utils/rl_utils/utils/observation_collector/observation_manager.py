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
    The ObservationManager class manages the collection and generation of observations from different units.

    It provides methods to initialize observation units, subscribe to topics, invalidate observations,
    wait for observations, and retrieve observations from collectors and generators.

    Args:
        ns (Namespace): The namespace object.
        obs_structur (List[ObservationCollectorUnit], optional): The list of observation unit types. Defaults to None.
        obs_unit_kwargs (dict, optional): Additional keyword arguments for observation units. Defaults to None.
        wait_for_obs (bool, optional): Whether to wait for observations to be available. Defaults to True.
        is_single_env (bool, optional): Whether the simulation environment is a single environment. Defaults to None.
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
        is_single_env: bool = None,
    ) -> None:
        """
        Initialize ObservationManager with namespace and observation structure.

        Args:
            ns (Namespace): The namespace object.
            obs_structur (List[ObservationCollectorUnit], optional): The list of observation unit types. Defaults to None.
            obs_unit_kwargs (dict, optional): Additional keyword arguments for observation units. Defaults to None.
            wait_for_obs (bool, optional): Whether to wait for observations to be available. Defaults to True.
            is_single_env (bool, optional): Whether the simulation environment is a single environment. Defaults to None.
        """
        self._ns = ns
        self._obs_structur = list(explore_hierarchy(obs_structur).keys())

        obs_unit_kwargs = obs_unit_kwargs or {}
        self._collectable_observations = {}
        self._subscribers = {}

        self._wait_for_obs = wait_for_obs
        self._is_single_env = is_single_env or "sim" in ns

        self._inititialize_units(obs_unit_kwargs=obs_unit_kwargs)
        self._init_units()

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
                (
                    str(collector.topic)
                    if "crowdsim_agents" in collector.topic and self._is_single_env
                    else str(
                        get_topic(
                            self._ns, collector.topic, collector.is_topic_agent_specific
                        )
                    )
                ),
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
        try:
            rospy.wait_for_message(
                str(
                    get_topic(
                        self._ns,
                        self._collectors[name].topic,
                        self._collectors[name].is_topic_agent_specific,
                    )
                ),
                self._collectors[name].msg_data_class,
                timeout=10,
            )
        except rospy.ROSException:
            rospy.logwarn(
                f"Waiting for observation '{name}' timed out. The observation may be stale."
            )

    def _get_collectable_observations(
        self, obs_dict: ObservationDict
    ) -> ObservationDict:
        """
        Retrieves the collectable observations from the collectors.

        Args:
            obs_dict (ObservationDict): A dictionary to store the observations.

        Returns:
            ObservationDict: A dictionary containing the collectable observations.
        """
        # Retrieve all observations from the collectors
        for name, observation in self._collectable_observations.items():
            if observation.stale:
                rospy.logdebug_throttle(1, f"Observation '{name}' IS STALE.")
                if self._wait_for_obs and self._collectors[name].up_to_date_required:
                    self._wait_for_observation(name)
            obs_dict[name] = observation.value

        self._invalidate_observations()
        return obs_dict

    def _get_generatable_observations(
        self, obs_dict: ObservationDict
    ) -> ObservationDict:
        """
        Generates observations from the generators and updates the observation dictionary.

        Args:
            obs_dict (ObservationDict): The observation dictionary to update.

        Returns:
            ObservationDict: The updated observation dictionary.
        """
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

    def get_observations(
        self, *args, **extra_observations: ObservationDict
    ) -> ObservationDict:
        """
        Get observations from all observation units.

        This method collects observations from all observation units and returns them as a dictionary.

        Args:
            *args: Additional positional arguments.
            **extra_observations: Additional keyword arguments representing extra observations/observations to be replaced.

        Returns:
            ObservationDict: Dictionary containing observations from all units.
        """
        obs_dict = {}

        self._get_collectable_observations(obs_dict)
        self._get_generatable_observations(obs_dict)

        for key, val in extra_observations.items():
            obs_dict[key] = val

        return obs_dict
