import os
from typing import Optional, Literal

from pydantic import BaseModel, model_validator


class StopTrainingOnThreshCfg(BaseModel):
    threshold_type: Literal["rew", "succ"] = "succ"
    threshold: float = 0.9
    verbose: int = 1


class PeriodicEvaluationCfg(BaseModel):
    n_eval_episodes: int = 40
    eval_freq: int = 20000
    max_num_moves_per_eps: int = 250


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


class CallbacksCfg(BaseModel):
    periodic_evaluation: Optional[PeriodicEvaluationCfg] = PeriodicEvaluationCfg()
    training_curriculum: Optional[TrainingCurriculumCfg] = TrainingCurriculumCfg()
    stop_training_on_threshold: Optional[StopTrainingOnThreshCfg] = (
        StopTrainingOnThreshCfg()
    )
