import functools
from typing import TYPE_CHECKING, Dict, List, Optional

import rospy
from rl_utils.state_container import SimulationStateContainer
from task_generator.shared import Namespace

from .collectors.base_collector import ObservationCollectorUnit
from .generators.base_generator import ObservationGeneratorUnit
from .traversal import explore_hierarchy
from .utils.topic import get_topic

if TYPE_CHECKING:
    from rl_utils.utils.observation_collector.generic_observation import (
        GenericObservation,
    )
    from rl_utils.utils.type_alias.observation import (
        ObservationDict,
        TypeObservationGeneric,
    )


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
    _obs_structur: List["TypeObservationGeneric"]
    _simulation_state_container: SimulationStateContainer
    _collectors: Dict[str, ObservationCollectorUnit]
    _generators: Dict[str, ObservationGeneratorUnit]
    _collectable_observations: Dict[str, "GenericObservation"]
    _subscribers: Dict[str, rospy.Subscriber]

    def __init__(
        self,
        ns: Namespace,
        obs_structur: List["TypeObservationGeneric"],
        simulation_state_container: SimulationStateContainer,
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
        self._simulation_state_container = simulation_state_container
        self._obs_structur = list(explore_hierarchy(obs_structur).keys())

        obs_unit_kwargs = obs_unit_kwargs or {}
        self._collectable_observations = {}
        self._subscribers = {}

        self._wait_for_obs = wait_for_obs
        self._is_single_env = is_single_env or "sim" in ns

        obs_unit_kwargs.update(
            {"ns": self._ns, "simulation_state_container": simulation_state_container}
        )

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
        Initializes the observation units by creating instances of `GenericObservation` for each collector
        and setting up ROS subscribers for each observation.

        This method performs the following steps:
        1. Imports the `GenericObservation` class from `rl_utils.utils.observation_collector.generic_observation`.
        2. Iterates over the `_collectors` dictionary.
        3. For each collector, creates an instance of `GenericObservation` with the initial message and preprocessing function.
        4. Stores the `GenericObservation` instance in the `_collectable_observations` dictionary.
        5. Sets up a ROS subscriber for each collector's topic, using the appropriate topic name and message data class.

        Note:
            If the collector's topic contains "crowdsim_agents" and the environment is single, the topic name is used directly.
            Otherwise, the topic name is generated using the `get_topic` function.

        Args:
            None

        Returns:
            None
        """
        import rl_utils.utils.observation_collector.generic_observation as gen_obs

        for collector in self._collectors.values():
            observation_container = gen_obs.GenericObservation(
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
        Invalidates all collectable observations.

        This method iterates through all the collectable observations and calls
        the `invalidate` method on each collector to mark them as invalid.
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
        self, obs_dict: "ObservationDict"
    ) -> "ObservationDict":
        """
        Retrieve and return all collectable observations.

        This method iterates through the collectable observations and checks if any
        of them are stale. If an observation is stale and up-to-date data is required,
        it waits for the observation to be updated. All observations are then added
        to the provided observation dictionary.

        Args:
            obs_dict (ObservationDict): The dictionary to populate with observations.

        Returns:
            ObservationDict: The updated dictionary containing all collectable observations.
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
        self,
        obs_dict: "ObservationDict",
        simulation_state_container: SimulationStateContainer,
    ) -> "ObservationDict":
        """
        Generates observations using the available generators and updates the observation dictionary.

        Args:
            obs_dict (ObservationDict): The dictionary containing current observations.
            simulation_state_container (SimulationStateContainer): The container holding the current state of the simulation.

        Returns:
            ObservationDict: The updated observation dictionary with generated observations.

        Raises:
            KeyError: If a generator fails to generate an observation, a warning is logged and the observation is set to None.
        """
        # Generate observations from the generators
        for generator in self._generators.values():
            try:
                obs_dict[generator.name] = generator.generate(
                    obs_dict=obs_dict,
                    simulation_state_container=simulation_state_container,
                )
            except KeyError as e:
                rospy.logwarn_once(
                    f"{e} \n Could not generate observation for '{generator.name}'."
                )
                obs_dict[generator.name] = None

        return obs_dict

    def get_observations(
        self,
        *args,
        **extra_observations: "ObservationDict",
    ) -> "ObservationDict":
        """
        Collects and returns a dictionary of observations.

        This method gathers observations from various sources, including
        collectable and generatable observations, and combines them with
        any additional observations provided.

        Args:
            simulation_state_container (Optional[SimulationStateContainer]):
                An optional container holding the state of the simulation.
            *args: Additional positional arguments.
            **extra_observations (ObservationDict):
                Additional observations to be included in the final dictionary.

        Returns:
            ObservationDict: A dictionary containing all collected observations.
        """
        obs_dict = {}

        self._get_collectable_observations(obs_dict)
        self._get_generatable_observations(
            obs_dict=obs_dict,
            simulation_state_container=self._simulation_state_container,
        )

        for key, val in extra_observations.items():
            obs_dict[key] = val

        return obs_dict
