from typing import Any, Dict, List

import rospy
from pedsim_msgs.msg import (
    AgentState,
    AgentStates,
)
from task_generator.shared import Namespace
from geometry_msgs.msg import Pose, Twist
from ..constants import TOPICS, OBS_DICT_KEYS
from .collector_unit import CollectorUnit


class PedsimStateCollectorUnit(CollectorUnit):
    """
    A class for collecting and managing pedestrian simulation state data.

    Attributes:
        _agent_states (List[AgentState]): A list of agent states.
        _agent_states_sub (rospy.Subscriber): A subscriber for agent states.

        _agent_poses (List[Pose]): A list of agent poses.
        _agent_twists (List[Twist]): A list of agent twists.
        _agent_types (List[str]): A list of agent types.
    """

    _agent_states: List[AgentState]
    _agent_states_sub: rospy.Subscriber

    _agent_poses: List[Pose]
    _agent_twists: List[Twist]
    _agent_types: List[str]

    def __init__(
        self, ns: Namespace, observation_manager: "ObservationCollector"
    ) -> None:
        """
        Initializes the PedsimStateCollectorUnit.

        Args:
            ns (Namespace): The namespace for the simulation.
            observation_manager (ObservationCollector): The observation manager containing the unit.
        """
        super().__init__(ns.simulation_ns, observation_manager)
        self._agent_states = AgentStates()
        self._agent_states_sub: rospy.Subscriber = None

        self._agent_poses = []
        self._agent_twists = []
        self._agent_types = []

    def init_subs(self):
        """
        Initializes the subscriber for agent states.
        """
        self._agent_states_sub = rospy.Subscriber(
            self._ns(TOPICS.PEDSIM_STATES), AgentStates, self._cb_pedsim_states
        )

    def get_observations(
        self, obs_dict: Dict[str, Any], *args, **kwargs
    ) -> Dict[str, Any]:
        """
        Retrieves the observations and updates the observation dictionary.

        Args:
            obs_dict (Dict[str, Any]): The observation dictionary.

        Returns:
            Dict[str, Any]: The updated observation dictionary.
        """

        obs_dict = super().get_observations(obs_dict, *args, **kwargs)
        obs_dict.update(
            {
                OBS_DICT_KEYS.PEDSIM_POSES: self._agent_poses,
                OBS_DICT_KEYS.PEDSIM_TWISTS: self._agent_twists,
                OBS_DICT_KEYS.PEDSIM_TYPES: self._agent_types,
            }
        )
        return obs_dict

    def _cb_pedsim_states(self, pedsim_states_msg: AgentStates) -> None:
        """
        Callback function for processing received pedestrian simulation states.

        Args:
            pedsim_states_msg (AgentStates): The received pedestrian simulation states message.
        """
        self._agent_states = pedsim_states_msg.agent_states
        self._agent_poses = PedsimStateCollectorUnit._get_agent_positions(
            self._agent_states
        )
        self._agent_twists = PedsimStateCollectorUnit._get_agent_twists(
            self._agent_states
        )
        self._agent_types = PedsimStateCollectorUnit._get_agent_types(
            self._agent_states
        )

    @staticmethod
    def _get_agent_positions(agent_states: List[AgentState]) -> List[Pose]:
        """
        Static method to extract agent positions from a list of agent states.

        Args:
            agent_states (List[AgentState]): A list of agent states.

        Returns:
            List[Pose]: A list of agent poses.
        """
        return [agent.pose for agent in agent_states]

    @staticmethod
    def _get_agent_twists(agent_states: List[AgentState]) -> List[Twist]:
        """
        Static method to extract agent twists from a list of agent states.

        Args:
            agent_states (List[AgentState]): A list of agent states.

        Returns:
            List[Twist]: A list of agent twists.
        """
        return [agent.twist for agent in agent_states]

    @staticmethod
    def _get_agent_types(agent_states: List[AgentState]) -> List[str]:
        """
        Static method to extract agent types from a list of agent states.

        Args:
            agent_states (List[AgentState]): A list of agent states.

        Returns:
            List[str]: A list of agent types.
        """
        return [agent.type for agent in agent_states]
