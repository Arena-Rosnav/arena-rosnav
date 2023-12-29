import functools
from typing import Any, Dict, List

import numpy as np

import rospy
from pedsim_msgs.msg import (
    AgentState,
    AgentStates,
)
from task_generator.shared import Namespace
from geometry_msgs.msg import Pose, Twist
from ..constants import TOPICS, OBS_DICT_KEYS
from .collector_unit import CollectorUnit

from pedsim_agents.utils import SemanticAttribute
import pedsim_msgs.msg as pedsim_msgs

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
    _semantic_sub: Dict[SemanticAttribute, rospy.Subscriber]

    _agent_poses: List[Pose]
    _agent_twists: List[Twist]
    _agent_types: List[str]

    _pedsim_semantic: Dict[SemanticAttribute, List[Any]]

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

        self._semantic_sub = dict()
        

        self._agent_poses = []
        self._agent_twists = []
        self._agent_types = []

        self._pedsim_semantic = {attribute: list() for attribute in SemanticAttribute}

    def init_subs(self):
        """
        Initializes the subscriber for agent states.
        """
        self._agent_states_sub = rospy.Subscriber(
            self._ns(TOPICS.PEDSIM_STATES), AgentStates, self._cb_pedsim_states
        )

        for attribute in SemanticAttribute:
            self._semantic_sub[attribute] = rospy.Subscriber(
                self._ns(TOPICS.PEDSIM_SEMANTIC),
                pedsim_msgs.SemanticData,
                functools.partialmethod(self._cb_pedsim_semantic, attribute)
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

    def _cb_pedsim_semantic(self, attribute: SemanticAttribute, data: pedsim_msgs.SemanticData):
        
        if attribute != SemanticAttribute(data.type):
            rospy.logwarn("mismatched semantic type")
            return
        
        layer = self._pedsim_semantic[attribute]
        layer = []

        for datum in list(data.points or []):
            layer.append(dict(
                x=datum.location.x,
                y=datum.location.y,
                evidence=datum.evidence
            ))
        

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
