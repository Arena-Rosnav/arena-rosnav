from typing import Dict, List, Optional

import rospy
from pydantic import BaseModel, Field, field_validator, model_validator
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
    robot_description: Optional[RobotYamlCfg] = Field(
        alias="Robot Yaml Description",
        default_factory=lambda: RobotYamlCfg.model_validate(
            load_yaml(get_robot_yaml_path(rospy.get_param("model")))
        ),
    )
    attach_full_range_laser: Optional[bool] = True
