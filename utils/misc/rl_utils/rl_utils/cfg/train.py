from pydantic import BaseModel, model_validator, ConfigDict
from rl_utils.cfg.sb3_cfg.robot import DiscreteAction
from rl_utils.utils.type_alias.observation import CustomDiscreteActionList
from rosnav_rl.cfg.agent import AgentCfg

from .sb3_cfg import SB3Cfg


class TrainingCfg(BaseModel):
    framework_cfg: SB3Cfg
    agent_cfg: AgentCfg

    model_config = ConfigDict(
        extra="forbid",  # Reject extra fields
        arbitrary_types_allowed=True,  # Allow custom types
    )

    # TODO: Maybe move this to a more general place - closer to the RobotCfg
    @model_validator(mode="after")
    def generate_custom_discrete_actions(self):
        if (
            self.agent_cfg.action_space.custom_discretization is not None
            and self.framework_cfg.robot is not None
        ):
            custom_discrete_actions_list: CustomDiscreteActionList = (
                self.agent_cfg.action_space.custom_discretization.generate_discrete_action_dict(
                    self.framework_cfg.robot.robot_description.actions.continuous.linear_range,
                    self.framework_cfg.robot.robot_description.actions.continuous.linear_range,
                )
            )
            self.framework_cfg.robot.robot_description.actions.discrete = [
                DiscreteAction(**action) for action in custom_discrete_actions_list
            ]
        return self
