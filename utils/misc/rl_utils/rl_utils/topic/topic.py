from rl_utils.topic import Topic, Namespace


# Example usage
if __name__ == "__main__":
    # Create a Topic using the new syntax
    topic: Topic = Namespace("sim_1") / Namespace("sim_1_jackal") / Topic("cmd_vel")
    print(topic)  # Output: /sim_1/sim_1_jackal/cmd_vel

    # You can also create topics step by step
    sim_namespace = Namespace("sim_1")
    robot_topic = sim_namespace / Namespace("robot_1")
    full_topic = robot_topic / "sensor_data"
    print(full_topic)  # Output: /sim_1/robot_1/sensor_data

    # The previous methods still work
    topic2 = Topic("odom", namespaces=["sim_2", "robot_2"])
    print(topic2)  # Output: /sim_2/robot_2/odom

    # You can also mix and match
    mixed_topic = (
        Namespace("sim_3") / Namespace("") / Topic("cmd_vel", namespaces=["custom_ns"])
    )
    print(mixed_topic)  # Output: /sim_3/custom_ns/robot_3/status

    t1 = Topic("odom", namespaces="/sim_1//robot_1")
    print(t1)  # Output: /sim_1/robot_1/odom

    t2 = Topic("/sim_1//robot_1/jackal/odom")
    print(t2)  # Output: /sim_1/robot_1/jackal/odom

    t3 = Namespace("sim_1") / "robot_1" / "jackal" / "odom"
    print(t3)  # Output: /sim_1/robot_1/jackal/odom

    t4 = Namespace("sim_1")(Namespace("robot_1"), "jackal", "odom")
    print(t4)  # Output: /sim_1/robot_1/jackal/odom

    t5 = Namespace("/sim_1/robot_1")
    t6 = t5("odom")
    print(t6)  # Output: /sim_1/robot_1/odom
