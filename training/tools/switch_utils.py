from typing import Tuple, Union

import gym
import rospkg
import yaml
from rl_utils.envs.switch_env import SwitchFlatlandEnv
from rl_utils.utils.drl_switch.constants import PLANNER_CONFIG_PATH
from rosnav.model.base_agent import BaseAgent
from stable_baselines3.common.utils import set_random_seed

from task_generator.shared import Namespace

from tools.general import initialize_config


def read_yaml(file_path: str) -> dict:
    with open(file_path, encoding="utf-8") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    return config


def _init_switch_env_fnc(
    ns: Union[str, Namespace],
    agent_description: BaseAgent,
    planner_configuration: dict,
    reward_fnc: str,
    max_steps_per_episode: int,
    seed: int = 0,
    init_by_call: bool = False,
    obs_unit_kwargs: dict = None,
    reward_fnc_kwargs: dict = None,
    task_generator_kwargs: dict = None,
):
    """
    Returns the switch environment function.

    Args:
        ns (Union[str, Namespace]): The namespace for the environment.
        agent_description (BaseAgent): The agent description.
        planner_configuration (dict): The planner configuration.
        reward_fnc (str): The reward function.
        max_steps_per_episode (int): The maximum number of steps per episode.
        seed (int, optional): The random seed. Defaults to 0.
        init_by_call (bool, optional): Whether to trigger initialization. Defaults to False.
        obs_unit_kwargs (dict, optional): The observation unit keyword arguments. Defaults to None.
        reward_fnc_kwargs (dict, optional): The reward function keyword arguments. Defaults to None.
        task_generator_kwargs (dict, optional): The task generator keyword arguments. Defaults to None.

    Returns:
        Union[gym.Env, gym.Wrapper]: The initialized switch environment.
    """

    def _init() -> Union[gym.Env, gym.Wrapper]:
        return SwitchFlatlandEnv(
            ns=ns,
            agent_description=agent_description,
            planner_configuration=planner_configuration,
            reward_fnc=reward_fnc,
            max_steps_per_episode=max_steps_per_episode,
            init_by_call=init_by_call,
            obs_unit_kwargs=obs_unit_kwargs,
            reward_fnc_kwargs=reward_fnc_kwargs,
            task_generator_kwargs=task_generator_kwargs,
        )

    set_random_seed(seed)
    return _init


def initialize_switch_config(
    paths: dict, config: dict, n_envs: int, debug_mode: bool
) -> dict:
    """
    Initializes the switch configuration.

    Args:
        paths (dict): The paths for loading and saving data.
        config (dict): The configuration parameters.
        n_envs (int): The number of environments.
        debug_mode (bool): Whether to enable debug mode.

    Returns:
        dict: The initialized switch configuration.
    """
    config.update({"planner_configuration": read_yaml(PLANNER_CONFIG_PATH)})

    config = initialize_config(
        paths=paths,
        config=config,
    )

    return config
