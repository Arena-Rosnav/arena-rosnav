from typing import Literal, Tuple, TypedDict

import rl_utils.cfg as cfg
import rl_utils.cfg.sb3_cfg.algorithms.ppo.args as ppo_args
import rosnav_rl.cfg as rl_cfg
import rosnav_rl.rl_agent as Rosnav_RL
import rospy
from rl_utils.cfg import SB3TrainingCfg
from rl_utils.state_container import SimulationStateContainer
from rl_utils.trainer.arena_trainer import ArenaTrainer
from rl_utils.utils.type_alias.observation import PathsDict
from rosnav_rl.rl_agent import RL_Agent
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.vec_env import VecEnv
from tools.config import load_training_config, unpack_framework_config
from tools.env_utils import make_envs
from tools.general import (
    create_directories,
    get_paths,
    print_base_model,
    write_config_yaml,
)
from tools.model_utils import init_sb3_callbacks, setup_wandb
from tools.states import get_arena_states


class StableBaselines3Trainer(ArenaTrainer):
    FRAMEWORK: Literal["SB3"] = "SB3"
    agent: RL_Agent
    paths: PathsDict
    simulation_state_container: SimulationStateContainer
    config: SB3TrainingCfg

    def __init__(self, config: SB3TrainingCfg) -> None:
        self.config = config

        print(f"________ STARTING TRAINING WITH:  {config.agent_cfg.name} ________\n")
        print_base_model(config)

        (
            general_cfg,
            callbacks_cfg,
            monitoring_cfg,
            task_cfg,
            robot_cfg,
            normalization_cfg,
            profiling_cfg,
        ) = unpack_framework_config(config.framework_cfg)
        agent_cfg: rl_cfg.AgentCfg = config.agent_cfg

        self.__setup_node(general_cfg.debug_mode)
        self.paths = self._setup_directories(
            agent_cfg=agent_cfg,
            callbacks_cfg=callbacks_cfg,
            monitoring_cfg=monitoring_cfg,
            general_cfg=general_cfg,
        )

        if not general_cfg.debug_mode:
            write_config_yaml(config.model_dump(), self.paths["config"])

        self.simulation_state_container = self._init_simulation_state_container(
            agent_cfg=agent_cfg,
            general_cfg=general_cfg,
            robot_cfg=robot_cfg,
            task_cfg=task_cfg,
        )
        self.agent = self._initialize_agent(
            agent_cfg=agent_cfg, sim_state_container=self.simulation_state_container
        )

        train_env, eval_env = self._setup_environment(
            agent_instance=self.agent,
            sim_state_container=self.simulation_state_container,
            agent_cfg=agent_cfg,
            general_cfg=general_cfg,
            callbacks_cfg=callbacks_cfg,
            normalization_cfg=normalization_cfg,
            monitoring_cfg=monitoring_cfg,
            profiling_cfg=profiling_cfg,
        )
        eval_cb = self._setup_callbacks(
            train_env=train_env,
            eval_env=eval_env,
            general_cfg=general_cfg,
            task_cfg=task_cfg,
            callbacks_cfg=callbacks_cfg,
        )

        # initialize model
        self.agent.initialize_model(
            env=train_env,
            no_gpu=general_cfg.no_gpu,
            tensorboard_log_path=(
                self.paths["tb"] if not general_cfg.debug_mode else None
            ),
        )

        self._setup_monitoring(config=config, agent_instance=self.agent)

        self.method_arguments = self._prepare_method_arguments(
            eval_cb=eval_cb, general_cfg=general_cfg
        )

    def __setup_node(self, debug_mode: bool):
        rospy.set_param("debug_mode", debug_mode)
        if debug_mode:
            rospy.init_node("debug_node", disable_signals=False)

    def _initialize_agent(
        self, agent_cfg: rl_cfg.AgentCfg, sim_state_container: SimulationStateContainer
    ) -> Rosnav_RL.RL_Agent:
        return Rosnav_RL.RL_Agent(
            agent_cfg=agent_cfg,
            simulation_state_container=sim_state_container,
        )

    def _setup_directories(
        self,
        agent_cfg: rl_cfg.AgentCfg,
        callbacks_cfg: cfg.CallbacksCfg,
        monitoring_cfg: cfg.MonitoringCfg,
        general_cfg: cfg.GeneralCfg,
    ):
        paths = get_paths(
            agent_name=agent_cfg.name,
            curriculum_file=callbacks_cfg.training_curriculum.curriculum_file,
        )
        if not general_cfg.debug_mode:
            create_directories(
                paths=paths,
                resume_name=agent_cfg.framework.model.resume,
                log_evaluation=monitoring_cfg.eval_metrics,
                use_wandb=monitoring_cfg.wandb,
            )
        return paths

    def _setup_environment(
        self,
        agent_instance: RL_Agent,
        sim_state_container: SimulationStateContainer,
        agent_cfg: rl_cfg.AgentCfg,
        general_cfg: cfg.GeneralCfg,
        callbacks_cfg: cfg.CallbacksCfg,
        normalization_cfg: cfg.NormalizationCfg,
        monitoring_cfg: cfg.MonitoringCfg,
        profiling_cfg: cfg.ProfilingCfg,
    ) -> Tuple[VecEnv, VecEnv]:
        return make_envs(
            rl_agent=agent_instance,
            simulation_state_container=sim_state_container,
            agent_cfg=agent_cfg,
            general_cfg=general_cfg,
            callback_cfg=callbacks_cfg,
            normalization_cfg=normalization_cfg,
            monitoring_cfg=monitoring_cfg,
            profiling_cfg=profiling_cfg,
        )

    def _setup_callbacks(
        self,
        train_env: VecEnv,
        eval_env: VecEnv,
        general_cfg: cfg.GeneralCfg,
        task_cfg: cfg.TaskCfg,
        callbacks_cfg: cfg.CallbacksCfg,
    ) -> EvalCallback:
        return init_sb3_callbacks(
            train_env=train_env,
            eval_env=eval_env,
            n_envs=general_cfg.n_envs,
            tm_modules=task_cfg.tm_modules,
            callback_cfg=callbacks_cfg,
            model_save_path=self.paths["model"],
            eval_log_path=self.paths["eval"],
            debug_mode=general_cfg.debug_mode,
        )

    def _setup_monitoring(self, config: SB3TrainingCfg, agent_instance: RL_Agent):
        if (
            not config.framework_cfg.general.debug_mode
            and config.framework_cfg.monitoring.wandb
        ):
            setup_wandb(train_cfg=config, rl_model=agent_instance.model)

    def _init_simulation_state_container(
        self,
        agent_cfg: rl_cfg.AgentCfg,
        general_cfg: cfg.GeneralCfg,
        robot_cfg: cfg.RobotCfg,
        task_cfg: cfg.TaskCfg,
    ) -> SimulationStateContainer:
        return get_arena_states(
            goal_radius=general_cfg.goal_radius,
            max_steps=general_cfg.max_num_moves_per_eps,
            is_discrete=agent_cfg.action_space.is_discrete,
            safety_distance=general_cfg.safety_distance,
            robot_cfg=robot_cfg,
            task_modules_cfg=task_cfg,
        )

    def _prepare_method_arguments(
        self, eval_cb: EvalCallback, general_cfg: cfg.GeneralCfg
    ) -> TypedDict:
        return ppo_args.TrainArguments(
            total_timesteps=general_cfg.n_timesteps,
            callback=eval_cb,
            progress_bar=general_cfg.show_progress_bar,
        )


def main():
    config = load_training_config("training_config.yaml")
    trainer = StableBaselines3Trainer(config)
    trainer.train()


if __name__ == "__main__":
    main()
