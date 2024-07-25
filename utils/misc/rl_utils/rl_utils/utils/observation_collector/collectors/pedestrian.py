from abc import ABC
from typing import Type

from crowdsim_agents.utils import SemanticAttribute, SemanticMsg

from .base_collector import ObservationCollectorUnit

__all__ = [
    "SemanticLayerCollector",
    "PedestrianLocationCollector",
    "PedestrianTypeCollector",
    "PedestrianVelXCollector",
    "PedestrianVelYCollector",
    "PedestrianSocialStateCollector",
]


class SemanticLayerCollector(ObservationCollectorUnit, ABC):
    name: str
    topic: str
    msg_data_class: Type[SemanticMsg] = SemanticMsg
    data_class: Type[SemanticMsg] = SemanticMsg
    is_topic_agent_specific: bool = False

    def preprocess(self, msg: SemanticMsg) -> SemanticMsg:
        return msg


class PedestrianLocationCollector(SemanticLayerCollector):
    """
    A class that collects the location of pedestrians in the environment.

    Attributes:
        name (str): The name of the collector, indicating that it collects pedestrian locations.
        topic (str): The ROS topic to subscribe to for pedestrian location information.
        is_topic_agent_specific (bool): Indicates whether the topic is specific to a particular agent.
    """

    name: str = SemanticAttribute.IS_PEDESTRIAN.value
    topic: str = f"/crowdsim_agents/semantic/{SemanticAttribute.IS_PEDESTRIAN.value}"


class PedestrianTypeCollector(SemanticLayerCollector):
    """
    A class for collecting pedestrian type observations.

    Attributes:
        name (str): The name of the collector, set to the value of SemanticAttribute.PEDESTRIAN_TYPE.
        topic (str): The topic to subscribe to for pedestrian type observations, set to "crowdsim_agents/semantic/PEDESTRIAN_TYPE".
        is_topic_agent_specific (bool): Indicates whether the topic is agent-specific or not, set to False.
    """

    name: str = SemanticAttribute.PEDESTRIAN_TYPE.value
    topic: str = f"/crowdsim_agents/semantic/{SemanticAttribute.PEDESTRIAN_TYPE.value}"


class PedestrianVelXCollector(SemanticLayerCollector):
    """
    A class that collects the x-velocity of pedestrians.

    Attributes:
        name (str): The name of the collector.
        topic (str): The topic to subscribe to for collecting pedestrian x-velocity.
        is_topic_agent_specific (bool): Indicates whether the topic is agent-specific or not.
    """

    name: str = SemanticAttribute.PEDESTRIAN_VEL_X.value
    topic: str = f"/crowdsim_agents/semantic/{SemanticAttribute.PEDESTRIAN_VEL_X.value}"


class PedestrianVelYCollector(SemanticLayerCollector):
    """
    A class that collects the y-velocity of pedestrians.

    Attributes:
        name (str): The name of the collector.
        topic (str): The topic to subscribe to for collecting pedestrian y-velocities.
        is_topic_agent_specific (bool): Indicates whether the topic is agent-specific or not.
    """

    name: str = SemanticAttribute.PEDESTRIAN_VEL_Y.value
    topic: str = f"/crowdsim_agents/semantic/{SemanticAttribute.PEDESTRIAN_VEL_Y.value}"


class PedestrianSocialStateCollector(SemanticLayerCollector):
    """
    A class that collects the social state of pedestrians.

    Attributes:
        name (str): The name of the collector, which is set to the value of SemanticAttribute.SOCIAL_STATE.
        topic (str): The topic to subscribe to for collecting the social state, which is set to "crowdsim_agents/semantic/social_state".
        is_topic_agent_specific (bool): Indicates whether the topic is agent-specific or not. In this case, it is set to False.
    """

    name: str = SemanticAttribute.SOCIAL_STATE.value
    topic: str = f"/crowdsim_agents/semantic/{SemanticAttribute.SOCIAL_STATE.value}"
