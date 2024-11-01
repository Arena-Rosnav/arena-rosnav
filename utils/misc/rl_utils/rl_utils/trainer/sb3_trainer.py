import os
from typing import Literal, Tuple, TypedDict

import rl_utils.cfg as cfg
import rl_utils.cfg.sb3_cfg.algorithms.ppo.args as ppo_args
import rl_utils.utils.paths as Paths
import rosnav_rl.cfg as rl_cfg
import rosnav_rl.rl_agent as Rosnav_RL
import rospy
from rl_utils.cfg import TrainingCfg
from rl_utils.state_container import SimulationStateContainer
from rl_utils.trainer.arena_trainer import ArenaTrainer, TrainerHook, TrainingFramework
from tools.config import load_training_config, unpack_sb3_config, ConfigUnpacker
from tools.env_utils import make_envs
from tools.model_utils import init_sb3_callbacks, setup_wandb
from tools.states import get_arena_states


class StableBaselines3Trainer(ArenaTrainer):
    FRAMEWORK: TrainingFramework = TrainingFramework.SB3

    def __init__(self, config: TrainingCfg):
        super().__init__(config)
        self._unpack_config()
        self.setup()

    def _unpack_config(self):
        """Unpack configuration into instance variables."""
        (
            self.general_cfg,
            self.monitoring_cfg,
            self.task_cfg,
            self.normalization_cfg,
            self.callbacks_cfg,
            self.profiling_cfg,
            self.robot_cfg,
        ) = unpack_sb3_config(self.config.framework_cfg)
        self.agent_cfg: rl_cfg.AgentCfg = self.config.agent_cfg

    def _register_framework_specific_hooks(self):
        """Register hooks specific to SB3 implementation."""
        # BEFORE_TRAINING
        self.register_hook(
            TrainerHook.BEFORE_TRAINING,
            lambda _: os.system(
                f"rosrun dynamic_reconfigure dynparam set /task_generator_server STAGED_curriculum {self.callbacks_cfg.training_curriculum.curriculum_file}"
            ),
        )
        self.register_hook(
            TrainerHook.BEFORE_TRAINING,
            lambda _: os.system(
                f"rosrun dynamic_reconfigure dynparam set /task_generator_server STAGED_index {self.callbacks_cfg.training_curriculum.current_stage}"
            ),
        )

    def _setup_agent(self) -> None:
        """Setup the RL agent."""
        self.agent = Rosnav_RL.RL_Agent(
            agent_cfg=self.agent_cfg,
            simulation_state_container=self.simulation_state_container,
        )

    def _setup_environment(self) -> None:
        """Setup training environment."""
        train_env, eval_env = make_envs(
            rl_agent=self.agent,
            simulation_state_container=self.simulation_state_container,
            agent_cfg=self.agent_cfg,
            general_cfg=self.general_cfg,
            callback_cfg=self.callbacks_cfg,
            normalization_cfg=self.normalization_cfg,
            monitoring_cfg=self.monitoring_cfg,
            profiling_cfg=self.profiling_cfg,
        )

        self.eval_cb = init_sb3_callbacks(
            train_env=train_env,
            eval_env=eval_env,
            n_envs=self.general_cfg.n_envs,
            tm_modules=self.task_cfg.tm_modules,
            callback_cfg=self.callbacks_cfg,
            model_save_path=self.paths[Paths.Agent],
            eval_log_path=self.paths[Paths.AgentEval],
            debug_mode=self.general_cfg.debug_mode,
        )

        self.agent.initialize_model(
            env=train_env,
            no_gpu=self.general_cfg.no_gpu,
            tensorboard_log_path=(
                self.paths[Paths.AgentTensorboard]
                if not self.general_cfg.debug_mode
                else None
            ),
        )

    def _setup_simulation_state_container(self) -> SimulationStateContainer:
        self.simulation_state_container = get_arena_states(
            goal_radius=self.general_cfg.goal_radius,
            max_steps=self.general_cfg.max_num_moves_per_eps,
            is_discrete=self.agent_cfg.action_space.is_discrete,
            safety_distance=self.general_cfg.safety_distance,
            robot_cfg=self.robot_cfg,
            task_modules_cfg=self.task_cfg,
        )

    def _setup_monitoring(self) -> None:
        """Setup monitoring tools."""
        if not self.general_cfg.debug_mode and self.monitoring_cfg.wandb:
            setup_wandb(train_cfg=self.config, rl_model=self.agent.model)

    def _setup_train_method_arguments(self) -> TypedDict:
        """Prepare arguments for training method."""
        self.method_arguments = ppo_args.TrainArguments(
            total_timesteps=self.general_cfg.n_timesteps,
            callback=self.eval_cb,
            progress_bar=self.general_cfg.show_progress_bar,
        )

    @property
    def is_debug_mode(self) -> bool:
        return self.general_cfg.debug_mode


def main():
    config = load_training_config("training_config.yaml")
    trainer = StableBaselines3Trainer(config)
    trainer.train()


if __name__ == "__main__":
    main()
