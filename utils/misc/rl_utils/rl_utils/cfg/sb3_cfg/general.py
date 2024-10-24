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

    @model_validator(mode="after")
    def propagate_safe_distance(self):
        # TODO: Perhaps adjust TaskManager and RobotManaer to take in the safety distance (or simulation_state_container) as an argument
        rospy.set_param(
            f"{SIMULATION_NAMESPACES.EVAL_PREFIX}_{rospy.get_param('model')}/safety_distance",
            self.safety_distance,
        )
        for idx in range(self.n_envs):
            rospy.set_param(
                f"{SIMULATION_NAMESPACES.SIM_PREFIX}{idx + 1}_{rospy.get_param('model')}/safety_distance",
                self.safety_distance,
            )
