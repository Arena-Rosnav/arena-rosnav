from typing import Union

import rosnav_rl
from pydantic import BaseModel, ConfigDict, model_validator
from rl_utils.cfg.arena_cfg.robot import DiscreteAction
from rl_utils.utils.type_alias.observation import CustomDiscreteActionList

from .arena_cfg import ArenaBaseCfg
from .sb3_cfg import ArenaSB3Cfg


class TrainingCfg(BaseModel):
    __version__ = "0.1.0"

    arena_cfg: Union[ArenaBaseCfg, ArenaSB3Cfg]
    agent_cfg: rosnav_rl.AgentCfg
    resume: bool = False

    model_config = ConfigDict(
        extra="forbid",  # Reject extra fields
        arbitrary_types_allowed=True,  # Allow custom types
    )

    # TODO: Maybe move this to a more general place - closer to the RobotCfg
    @model_validator(mode="after")
    def generate_custom_discrete_actions(self):
        if (
            self.agent_cfg.action_space.custom_discretization is not None
            and self.arena_cfg.robot is not None
        ):
            custom_discrete_actions_list: CustomDiscreteActionList = (
                self.agent_cfg.action_space.custom_discretization.generate_discrete_from_box_dict(
                    self.arena_cfg.robot.robot_description.actions.continuous.linear_range,
                    self.arena_cfg.robot.robot_description.actions.continuous.linear_range,
                )
            )
            self.arena_cfg.robot.robot_description.actions.discrete = [
                DiscreteAction(**action) for action in custom_discrete_actions_list
            ]
        return self

    @model_validator(mode="after")
    def validate_resume(self):
        if self.resume:
            assert (
                self.agent_cfg.name is not None
            ), "Agent name must be provided for resume!"
            assert (
                self.agent_cfg.framework.algorithm.checkpoint is not None
            ), "Checkpoint must be provided for resume!"
        return self
