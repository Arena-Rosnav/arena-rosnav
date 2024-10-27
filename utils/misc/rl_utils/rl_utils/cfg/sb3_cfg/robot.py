from typing import Dict, List, Optional

import rospy
from pydantic import BaseModel, model_validator, field_validator
from rosnav_rl.utils.utils import get_robot_yaml_path, load_yaml


class DiscreteAction(BaseModel):
    name: str
    linear: float
    angular: float


class ContinuousActionCfg(BaseModel):
    linear_range: List[float]
    angular_range: List[float]


class ActionsCfg(BaseModel):
    discrete: List[DiscreteAction]
    continuous: ContinuousActionCfg


class LaserCfg(BaseModel):
    angle: Dict[str, float]
    num_beams: int
    range: float
    update_rate: int


class RobotYamlCfg(BaseModel):
    robot_model: str
    robot_radius: float
    robot_base_frame: str
    robot_sensor_frame: str
    is_holonomic: bool
    actions: ActionsCfg
    laser: LaserCfg


class RobotCfg(BaseModel):
    robot_description: Optional[RobotYamlCfg] = None
    attach_full_range_laser: Optional[bool] = True

    @model_validator(mode="after")
    def read_robot_description(self):
        self.robot_description = RobotYamlCfg.model_validate(
            load_yaml(get_robot_yaml_path(rospy.get_param("model")))
        )

    @field_validator("robot_description", mode="before")
    @classmethod
    def check_robot_description(cls, v):
        if v is None:
            model = rospy.get_param("model")
            return RobotYamlCfg.model_validate(load_yaml(get_robot_yaml_path(model)))
        return v
