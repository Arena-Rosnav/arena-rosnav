import contextlib
import os

import rospy
from rosnav.utils.utils import get_actions_from_robot_yaml
from task_generator.shared import Namespace

from .general import generate_discrete_action_dict


def populate_ros_params(params: dict, paths: dict):
    # general params
    rospy.set_param("tm_robots", params["tm_robots"])
    rospy.set_param("tm_obstacles", params["tm_obstacles"])
    rospy.set_param("tm_modules", params["tm_modules"])

    rospy.set_param("training_config_path", paths["config"])

    robot_name = rospy.get_param("model")
    rospy.set_param("goal_radius", params["goal_radius"])
    rospy.set_param(f"{robot_name}/safety_distance", params["safety_distance"])

    # discrete actions
    if "action_space" in params["rl_agent"]:
        is_discrete = params["rl_agent"]["action_space"]["discrete"]
        rospy.set_param(
            "is_action_space_discrete",
            is_discrete,
        )
        if is_discrete:
            populate_discrete_action_space(params)

    tmp_params: dict = params["rl_agent"].copy()
    tmp_params.pop("resume")
    if tmp_params:
        rospy.set_param("rl_agent", tmp_params)

    # populate laser params
    populate_laser_params(params)

    # populate rgbd params
    populate_rgbd_params(params)

    curriculum_file = params["callbacks"]["training_curriculum"][
        "training_curriculum_file"
    ]
    staged_idx = params["callbacks"]["training_curriculum"]["curr_stage"]

    # shell command
    os.system(
        f"rosrun dynamic_reconfigure dynparam set /task_generator_server STAGED_curriculum {curriculum_file}"
    )
    os.system(
        f"rosrun dynamic_reconfigure dynparam set /task_generator_server STAGED_index {staged_idx}"
    )


def populate_laser_params(params: dict, agent_namespace: str = None):
    agent_topic_prefix = f"{agent_namespace}" if agent_namespace else ""
    agent_topic_prefix = Namespace(agent_topic_prefix)

    with contextlib.suppress(KeyError):
        rospy.set_param(
            agent_topic_prefix("laser/reduce_num_beams"),
            params["rl_agent"]["laser"]["reduce_num_beams"]["enabled"],
        )
    with contextlib.suppress(KeyError):
        rospy.set_param(
            agent_topic_prefix("laser/full_range_laser"),
            params["rl_agent"]["laser"]["full_range_laser"],
        )
    with contextlib.suppress(KeyError):
        rospy.set_param(
            agent_topic_prefix("laser/reduced_num_laser_beams"),
            params["rl_agent"]["laser"]["reduce_num_beams"]["num_beams"],
        )


def populate_rgbd_params(params: dict):
    with contextlib.suppress(KeyError):
        rospy.set_param("rgbd/enabled", params["rl_agent"]["rgbd"]["enabled"])


def populate_discrete_action_space(params: dict, agent_namespace: str = None):
    robot_model = rospy.get_param("model")
    actions = get_actions_from_robot_yaml(robot_model)

    custom_disc_params = params["rl_agent"]["action_space"]["custom_discretization"]

    if custom_disc_params["enabled"]:
        buckets_linear_vel, buckets_angular_vel = (
            custom_disc_params["buckets_linear_vel"],
            custom_disc_params["buckets_angular_vel"],
        )

        action_range = actions["continuous"]

        discrete_actions_dict = generate_discrete_action_dict(
            linear_range=action_range["linear_range"],
            angular_range=action_range["angular_range"],
            num_linear_actions=buckets_linear_vel,
            num_angular_actions=buckets_angular_vel,
        )
    else:
        discrete_actions_dict = actions["discrete"]

    agent_topic_prefix = f"{agent_namespace}/" if agent_namespace else ""
    rospy.set_param(f"{agent_topic_prefix}actions/discrete", discrete_actions_dict)


def populate_ros_configs(config):
    rospy.set_param("debug_mode", config["debug_mode"])


def set_space_encoder(config):
    rospy.set_param(
        "space_encoder",
        (
            "StackedEncoder"
            if config["rl_agent"]["frame_stacking"]["enabled"]
            else "DefaultEncoder"
        ),
    )
