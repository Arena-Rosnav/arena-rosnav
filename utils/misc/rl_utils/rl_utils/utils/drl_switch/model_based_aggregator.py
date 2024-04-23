import yaml
import rospy

from rl_utils.utils.drl_switch.constants import (
    MBF_COMPATIBLE_TYPE,
    PLANNER_CONFIG_PATH,
    PLANNER_PARAM_PATH,
    ROBOT_SPECIFIC_PATH,
)


def extract_planner_names(planners: dict):
    """
    Extracts the names of the planners from the given dictionary.

    Args:
        planners (dict): A dictionary containing information about the planners.

    Returns:
        list: A list of unique planner names that are compatible with MBF_COMPATIBLE_TYPE.LOCAL.
    """
    name_list = [planner_dict["name"] for planner_dict in planners]
    return list(set(name_list) & set(MBF_COMPATIBLE_TYPE.LOCAL.__dict__.keys()))


def set_planner_params(ns: str, config: dict):
    """
    Set planner parameters based on the given configuration.

    Args:
        ns (str): The namespace for the parameter.
        config (dict): A dictionary containing the configuration parameters.

    Returns:
        None
    """
    for key, value in config.items():
        if value and key != "controllers":
            rospy.set_param(f"{ns}{key}", value)
        elif key == "controllers":
            # update controllers list
            controllers = rospy.get_param(f"{ns}/{key}", [])

            controller = value[0]
            if not any(c.get("name") == controller.get("name") for c in controllers):
                controllers.append(controller)

            rospy.set_param(f"{ns}controllers", controllers)
        else:
            continue


def set_robot_specific_params(ns: str, config: dict):
    """
    Set robot-specific parameters in the ROS parameter server.

    Args:
        ns (str): The namespace for the parameters.
        config (dict): A dictionary containing the parameters to be set.

    Returns:
        None
    """
    for key, value in config.items():
        params = rospy.get_param(f"{ns}{key}")
        params.update(value)
        rospy.set_param(f"{ns}{key}", params)


def read_yaml(file_path: str) -> dict:
    with open(file_path, encoding="utf-8") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    return config


def populate_params(ns: str, planners: list):
    """
    Populates default planner parameters for a given namespace.

    Args:
        ns (str): The namespace to populate planner parameters for.
        planners (dict): A dict of planner names and configs.

    Returns:
        None
    """

    files = {
        planner: PLANNER_PARAM_PATH(planner)
        for planner in extract_planner_names(planners)
    }

    for _, file in files.items():
        config = read_yaml(file)
        set_planner_params(ns, config)


def populate_robot_specific_params(ns: str, planners: list):
    """
    Populates robot-specific parameters for the given planners.

    Args:
        ns (str): The namespace for the parameters.
        planners (list): A list of planner names.

    Returns:
        None
    """
    model = rospy.get_param("/model")

    files = {
        planner: ROBOT_SPECIFIC_PATH(model, planner)
        for planner in extract_planner_names(planners)
    }

    for _, file in files.items():
        config = read_yaml(file)
        set_robot_specific_params(ns, config)


def main(ns: str = "") -> None:
    with open(PLANNER_CONFIG_PATH, encoding="utf-8") as f:
        planner_config = yaml.load(f, Loader=yaml.FullLoader)
    planners = planner_config["planners"]

    populate_params(ns, planners)
    populate_robot_specific_params(ns, planners)


if __name__ == "__main__":
    # get the namespace from the launch file
    rospy.init_node("model_based_aggregator", disable_signals=True)
    main(rospy.get_namespace())
