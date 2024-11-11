from dataclasses import dataclass
from typing import Any, Tuple

import rl_utils.utils.paths as Paths
import rosnav_rl.rl_agent as Rosnav_RL
from rl_utils.cfg import TrainingCfg
from rl_utils.trainer.arena_trainer import (
    ArenaTrainer,
    TrainingArguments,
    TrainingFramework,
    TrainingHookStages,
)
from rl_utils.utils.dynamic_reconfigure import set_dynamic_reconfigure_parameter
from stable_baselines3.common.vec_env.base_vec_env import VecEnv
from tools.config import SB3ConfigManager, load_training_config
from tools.env_utils import make_envs
from tools.model_utils import init_sb3_callbacks, setup_wandb
from tools.states import get_arena_states


@dataclass
class SB3TrainingArguments(TrainingArguments):
    total_timesteps: int
    callback: Any
    progress_bar: bool = True


@dataclass
class SB3Environment:
    train_env: VecEnv
    eval_env: VecEnv


class StableBaselines3Trainer(ArenaTrainer):
    """
    Trainer class for Stable Baselines 3 (SB3) framework.

    Attributes:
        FRAMEWORK (TrainingFramework): The training framework used (SB3).
    """

    FRAMEWORK = TrainingFramework.SB3
    config_manager: SB3ConfigManager
    environment: SB3Environment

    def __init__(self, config: TrainingCfg) -> None:
        self.config_manager = SB3ConfigManager(config)
        self.__unpack_config()
        super().__init__(config, config.resume)

    def __unpack_config(self) -> None:
        """Unpack SB3-specific configuration."""
        self.general_cfg = self.config_manager.fields.general
        self.monitoring_cfg = self.config_manager.fields.monitoring
        self.task_cfg = self.config_manager.fields.task
        self.normalization_cfg = self.config_manager.fields.normalization
        self.callbacks_cfg = self.config_manager.fields.callbacks
        self.profiling_cfg = self.config_manager.fields.profiling
        self.robot_cfg = self.config_manager.fields.robot
        self.agent_cfg = self.config_manager.fields.agent

    def _register_framework_specific_hooks(self) -> None:
        """Register SB3-specific training hooks."""
        TASK_GEN_SERVER_NODE = "task_generator_server"
        CURRICULUM_PARAM = "STAGED_curriculum"
        CURRICULUM_INDEX_PARAM = "STAGED_index"

        curriculum_file = self.callbacks_cfg.training_curriculum.curriculum_file
        current_stage = self.callbacks_cfg.training_curriculum.current_stage

        def _set_curriculum_file(_):
            return set_dynamic_reconfigure_parameter(
                TASK_GEN_SERVER_NODE, CURRICULUM_PARAM, curriculum_file
            )

        def _set_curriculum_stage(_):
            return set_dynamic_reconfigure_parameter(
                TASK_GEN_SERVER_NODE, CURRICULUM_INDEX_PARAM, current_stage
            )

        self.hook_manager.register(
            TrainingHookStages.BEFORE_SETUP,
            [_set_curriculum_file, _set_curriculum_stage],
        )

    def _load_impl(self, *args, **kwargs) -> None:
        checkpoint = self.agent_cfg.framework.model.resume.checkpoint

        self.agent.model.load(
            path=(self.paths[Paths.Agent].path / checkpoint),
            env=self.environment.train_env,
        )

    def _setup_agent(self) -> None:
        """Initialize the RL agent with configuration."""
        self.agent = Rosnav_RL.RL_Agent(
            agent_cfg=self.agent_cfg,
            simulation_state_container=self.simulation_state_container,
        )

    def _setup_environment(self) -> None:
        """Set up training and evaluation environments."""
        self._create_environments()
        self._setup_callbacks(self.environment.train_env, self.environment.eval_env)
        self._complete_model_initialization(self.environment.train_env)

    def _create_environments(self) -> None:
        """Create training and evaluation environments."""
        self.environment = SB3Environment(
            *make_envs(
                rl_agent=self.agent,
                simulation_state_container=self.simulation_state_container,
                agent_cfg=self.agent_cfg,
                general_cfg=self.general_cfg,
                callback_cfg=self.callbacks_cfg,
                normalization_cfg=self.normalization_cfg,
                monitoring_cfg=self.monitoring_cfg,
                profiling_cfg=self.profiling_cfg,
            )
        )

    def _setup_callbacks(self, train_env: VecEnv, eval_env: VecEnv) -> None:
        """Initialize training callbacks."""
        self.eval_cb = init_sb3_callbacks(
            train_env=train_env,
            eval_env=eval_env,
            n_envs=self.general_cfg.n_envs,
            tm_modules=self.task_cfg.tm_modules,
            callback_cfg=self.callbacks_cfg,
            model_save_path=self.paths[Paths.Agent].path,
            eval_log_path=self.paths[Paths.AgentEval].path,
            debug_mode=self.general_cfg.debug_mode,
        )

    def _setup_simulation_state_container(self) -> None:
        """Initialize simulation state container with configuration."""
        self.simulation_state_container = get_arena_states(
            goal_radius=self.general_cfg.goal_radius,
            max_steps=self.general_cfg.max_num_moves_per_eps,
            is_discrete=self.agent_cfg.action_space.is_discrete,
            safety_distance=self.general_cfg.safety_distance,
            robot_cfg=self.robot_cfg,
            task_modules_cfg=self.task_cfg,
        )

    def _setup_monitoring(self) -> None:
        """Set up monitoring tools if not in debug mode."""
        if not self.general_cfg.debug_mode and self.monitoring_cfg.wandb:
            setup_wandb(train_cfg=self.config, rl_model=self.agent.model)

    def _train_impl(self, *args, **kwargs) -> None:
        """Implementation of training logic."""
        self.agent.model.train(
            **SB3TrainingArguments(
                total_timesteps=self.general_cfg.n_timesteps,
                callback=self.eval_cb,
                progress_bar=self.general_cfg.show_progress_bar,
            ).to_dict()
        )

    def _complete_model_initialization(self, train_env: VecEnv) -> None:
        """Initialize the agent's model with environment."""
        tensorboard_log_path = (
            self.paths[Paths.AgentTensorboard].path
            if not self.general_cfg.debug_mode
            else None
        )
        resume_model_file = (
            None
            if not self.is_resume
            else self.paths[Paths.Agent].path
            / self.agent_cfg.framework.model.resume.checkpoint
        )

        self.agent.initialize_model(
            env=train_env,
            no_gpu=self.general_cfg.no_gpu,
            tensorboard_log_path=tensorboard_log_path,
            resume_model_file=resume_model_file,
        )


def main():
    config = load_training_config("training_config.yaml")
    trainer = StableBaselines3Trainer(config)
    trainer.train()


if __name__ == "__main__":
    main()
