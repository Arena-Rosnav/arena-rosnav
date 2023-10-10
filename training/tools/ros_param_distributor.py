import contextlib

import rospy
from rosnav.utils.utils import get_actions_from_robot_yaml

from .general import generate_discrete_action_dict


def populate_ros_params(params: dict):
    # general params
    rospy.set_param("task_mode", params["task_mode"])

    is_discrete = params["rl_agent"]["action_space"]["custom_discretization"]["enabled"]
    rospy.set_param(
        "is_action_space_discrete",
        is_discrete,
    )

    rospy.set_param("goal_radius", params["goal_radius"])

    # discrete actions
    if is_discrete:
        populate_discrete_action_space(params)

    # populate laser params
    enable_frame_stacking, enable_reduced_laser = populate_laser_params(params)

    rospy.set_param(
        "space_encoder",
        determine_space_encoder(enable_frame_stacking, enable_reduced_laser),
    )


def populate_laser_params(params: dict):
    with contextlib.suppress(KeyError):
        rospy.set_param(
            "laser/reduce_num_beams",
            params["rl_agent"]["laser"]["reduce_num_beams"]["enabled"],
        )
    with contextlib.suppress(KeyError):
        rospy.set_param(
            "laser/full_range_laser", params["rl_agent"]["laser"]["full_range_laser"]
        )
    with contextlib.suppress(KeyError):
        rospy.set_param(
            "laser/reduced_num_laser_beams",
            params["rl_agent"]["laser"]["reduce_num_beams"]["num_beams"],
        )

    enable_frame_stacking, enable_reduced_laser = False, False
    with contextlib.suppress(KeyError):
        enable_frame_stacking = params["rl_agent"]["frame_stacking"]["enabled"]
    with contextlib.suppress(KeyError):
        enable_reduced_laser = params["rl_agent"]["laser"]["reduce_num_beams"][
            "enabled"
        ]
    return enable_frame_stacking, enable_reduced_laser


def populate_discrete_action_space(params: dict):
    robot_model = rospy.get_param("model")

    custom_disc_params = params["rl_agent"]["action_space"]["custom_discretization"]
    buckets_linear_vel, buckets_angular_vel = (
        custom_disc_params["buckets_linear_vel"],
        custom_disc_params["buckets_angular_vel"],
    )

    actions = get_actions_from_robot_yaml(robot_model)
    action_range = actions["continuous"]

    discrete_actions_dict = generate_discrete_action_dict(
        linear_range=action_range["linear_range"],
        angular_range=action_range["angular_range"],
        num_linear_actions=buckets_linear_vel,
        num_angular_actions=buckets_angular_vel,
    )

    rospy.set_param("actions/discrete", discrete_actions_dict)


def determine_space_encoder(frame_stacking: bool, reduced_laser: bool):
    if frame_stacking and not reduced_laser:
        return "StackedEncoder"
    elif not frame_stacking and reduced_laser:
        return "ReducedLaserEncoder"
    elif frame_stacking and reduced_laser:
        return "StackedReducedLaserEncoder"
    else:
        return "DefaultEncoder"


def populate_ros_configs(config):
    rospy.set_param("debug_mode", config["debug_mode"])


def set_space_encoder(config):
    rospy.set_param(
        "space_encoder",
        "StackedEncoder"
        if config["rl_agent"]["frame_stacking"]["enabled"]
        else "DefaultEncoder",
    )
