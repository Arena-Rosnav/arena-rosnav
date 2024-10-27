import rospy
from pydantic import BaseModel, model_validator, field_validator
from tools.constants import SIMULATION_NAMESPACES


class GeneralCfg(BaseModel):
    """
    General configuration settings for reinforcement learning environments.

    Attributes:
        debug_mode (bool): Flag to enable or disable debug mode. Default is False.
        n_envs (int): Number of environments to run in parallel. Default is 1.
        max_num_moves_per_eps (int): Maximum number of moves allowed per episode. Default is 150.
        goal_radius (float): Radius around the goal within which the agent is considered to have reached the goal. Default is 0.4.
        safety_distance (float): Minimum safety distance to be maintained by the agent (considers robot radius). Default is 1.0.
    """

    debug_mode: bool = False
    no_gpu: bool = False
    n_envs: int = 1
    n_timesteps: int = 4000000
    max_num_moves_per_eps: int = 150
    goal_radius: float = 0.4
    safety_distance: float = 1.0
    show_progress_bar: bool = False

    @field_validator(
        "safety_distance", "goal_radius", "n_envs", "max_num_moves_per_eps"
    )
    @classmethod
    def check_safety_distance(cls, v: float) -> float:
        if v < 0:
            raise ValueError(f"{cls} cannot be negative.")
        return v
