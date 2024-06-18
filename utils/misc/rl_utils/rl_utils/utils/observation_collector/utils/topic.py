from task_generator.shared import Namespace


def get_topic(ns: Namespace, topic: str, agent_specific: bool) -> str:
    """
    Get the topic name.

    Args:
        topic (Namespace): The topic name.
        agent_specific (bool): Whether the topic is agent specific.

    Returns:
        str: The topic name.

    """
    return ns(topic) if agent_specific else ns.simulation_ns(topic)
