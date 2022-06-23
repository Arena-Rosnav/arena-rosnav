import rospy
import rospkg
import os


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