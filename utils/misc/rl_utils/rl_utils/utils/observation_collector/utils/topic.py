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


if __name__ == "__main__":
    ns = Namespace("/test/test_robot")
    print(get_topic(ns, "/test_topic", False))  # expect "/test/test_topic"
    print(get_topic(ns, "test_topic", False))  # expect "/test/test_topic"
