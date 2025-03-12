import os
from typing import Literal, Optional

from pydantic import BaseModel, model_validator
from rosnav_rl.model.stable_baselines3.cfg import CallbacksCfg


class TrainingCurriculumCfg(BaseModel):
    curriculum_file: str = "default.yaml"
    current_stage: int = 0
    threshold_type: Literal["rew", "succ"] = "succ"
    upper_threshold: float = 0.9
    lower_threshold: float = 0.3

    @model_validator(mode="after")
    def propagate_stage(self):
        # shell command
        os.system(
            f"rosrun dynamic_reconfigure dynparam set /task_generator_server STAGED_curriculum {self.curriculum_file}"
        )
        os.system(
            f"rosrun dynamic_reconfigure dynparam set /task_generator_server STAGED_index {self.current_stage}"
        )
        return self


class ArenaCallbacksCfg(CallbacksCfg):
    training_curriculum: Optional[TrainingCurriculumCfg] = TrainingCurriculumCfg()
