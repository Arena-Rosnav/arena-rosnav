import rospy
import rospkg
import os
import yaml
from gym import spaces
import numpy as np


def get_robot_default_settings_path():
    robot_model = rospy.get_param("model")

    return os.path.join(
        rospkg.RosPack().get_path("arena-simulation-setup"),
        "robot",
        robot_model,
        "default_settings.yaml",
    )


def get_default_hyperparams_path():
    return os.path.join(
        rospkg.RosPack().get_path("training"),
        "configs",
        "hyperparameters",
        "default.json",
    )


def get_trained_models_path():
    return os.path.join(
        rospkg.RosPack().get_path("rosnav"), "agents"
    )


def stack_spaces(*ss):
    low = []
    high = []
    for space in ss:
        low.extend(space.low.tolist())
        high.extend(space.high.tolist())
    return spaces.Box(np.array(low).flatten(), np.array(high).flatten())


def read_robot_model_yaml(robot_yaml_path):
    with open(robot_yaml_path, "r") as file:
        robot_data = yaml.safe_load(file)

        # get laser related information
        for plugin in robot_data["plugins"]:
            if plugin["type"] == "Laser" and plugin["name"] == "static_laser":
                laser_angle_min = plugin["angle"]["min"]
                laser_angle_max = plugin["angle"]["max"]
                laser_angle_increment = plugin["angle"]["increment"]
                
                # Num Laser Beams, Range
                return (
                    laser_angle_min,
                    laser_angle_max,
                    laser_angle_increment,
                    int(round((laser_angle_max - laser_angle_min) / laser_angle_increment)), 
                    plugin["range"],
                    plugin["update_rate"]
                )


def read_robot_default_settings(settings_yaml_path, is_action_space_discrete):
    with open(settings_yaml_path, "r") as fd:
        setting_data = yaml.safe_load(fd)
        robot = setting_data["robot"]

        assert not (robot["holonomic"] and is_action_space_discrete), "Discrete action space currently not supported for holonomic robots"

        actions = robot["discrete_actions"] if is_action_space_discrete else robot["continuous_actions"]

        return robot["holonomic"], actions

def create_params_for_robot(robot_yaml_path, settings_yaml_path, is_action_space_discrete):
    (laser_angle_min, 
    laser_angle_max, 
    laser_angle_increment, 
    laser_num_beams, 
    laser_range, 
    laser_update_rate) = read_robot_model_yaml(robot_yaml_path)
    is_holonomic, actions = read_robot_default_settings(settings_yaml_path, is_action_space_discrete)

    rospy.set_param("laser_angle_min", laser_angle_min)
    rospy.set_param("laser_angle_max", laser_angle_max)
    rospy.set_param("laser_angle_increment", laser_angle_increment)
    rospy.set_param("laser_num_beams", laser_num_beams)
    rospy.set_param("is_holonomic", is_holonomic)
    rospy.set_param("laser_max_range", laser_range)
    rospy.set_param("laser_update_rate", laser_update_rate)
    rospy.set_param("actions", actions)
    rospy.set_param("is_action_space_discrete", is_action_space_discrete)