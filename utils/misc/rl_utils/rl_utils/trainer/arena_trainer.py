import sys
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

import rl_utils.cfg as arena_cfg
import rl_utils.utils.paths as Paths
import rosnav_rl
import rospy
from rl_utils.tools.general import (
    print_base_model,
    setup_debug_node,
    setup_paths_dictionary,
    write_config_yaml,
)
from rl_utils.tools.states import get_arena_states
from rl_utils.utils.hooks import HookManager, TrainingHookStages, bind_hooks
from rl_utils.utils.type_alias.observation import EnvironmentType, PathsDict
from rosnav_rl.rl_agent import RL_Agent
from rosnav_rl.states import SimulationStateContainer
from rosnav_rl.utils.type_aliases import SupportedRLFrameworks


@dataclass
class TrainingArguments:
    """Arguments for training."""

    def to_dict(self) -> dict:
        return self.__dict__


class ArenaTrainer(ABC):
    """
    Abstract base class for reinforcement learning trainers in the Arena-Rosnav framework.

    ArenaTrainer handles the setup, training, and management of RL agents and their environments.
    It uses a hook-based system to manage the training workflow and provides a structured approach
    to implementing different RL frameworks.

    Attributes:
        __framework (SupportedRLFrameworks): The RL framework being used (e.g., SB3, RLlib).
        config (TrainingCfg): Configuration settings for the training process.
        paths (PathsDict): Dictionary containing paths for model saving, logging, etc.
        simulation_state_container (SimulationStateContainer): Container for simulation state data.
        agent (RL_Agent): The reinforcement learning agent being trained.
        environment (EnvironmentType): The training environment for the agent.
        hook_manager (HookManager): Manager for registering and executing hooks at various training stages.

    Methods:
        train: Executes the training process for the RL agent.
        save: Saves the trained model to a specified checkpoint.
        close: Cleans up resources and exits gracefully.

    The class uses a hook system through the @bind_hooks decorator to allow for
    custom callbacks at different stages of the training process.

    Implementing classes must provide concrete implementations for:
    - _setup_agent: Initialize the RL agent
    - _setup_environment: Create the training environment
    - _setup_monitoring: Set up monitoring and logging tools
    """

    __framework: SupportedRLFrameworks

    config: arena_cfg.TrainingCfg
    paths: PathsDict

    simulation_state_container: SimulationStateContainer
    agent: RL_Agent
    environment: EnvironmentType

    hook_manager: HookManager = HookManager()

    @bind_hooks(before_stage=TrainingHookStages.ON_INIT)
    def __init__(self, config: arena_cfg.TrainingCfg, resume: bool = False) -> None:
        """
        Initializes the ArenaTrainer with the given configuration.

        Args:
            config (TrainingCfg): The configuration object for training.
        """
        self.config = config
        self.__resume = resume

        self._register_default_hooks()
        self._register_framework_specific_hooks()
        self._setup_trainer()

    @bind_hooks(
        before_stage=TrainingHookStages.BEFORE_SETUP,
        after_stage=TrainingHookStages.AFTER_SETUP,
    )
    def _setup_trainer(self) -> None:
        """
        Sets up the trainer by sequentially executing the following setup steps:
        1. Sets up simulation state container
        2. Sets up agent state container
        3. Sets up the agent
        4. Sets up the environment
        5. Sets up monitoring

        This method orchestrates the initialization process in a structured order
        to ensure all components are properly initialized before training.

        Returns:
            None
        """
        setup_steps = [
            self._setup_simulation_state_container,
            self._setup_agent_state_container,
            self._setup_agent,
            self._setup_environment,
        ]

        for step in setup_steps:
            step()

    @bind_hooks(
        before_stage=TrainingHookStages.BEFORE_TRAINING,
        after_stage=TrainingHookStages.AFTER_TRAINING,
    )
    def train(self, *args, **kwargs) -> None:
        """
        Execute the training process for the reinforcement learning agent.

        This method acts as a wrapper that calls the internal implementation
        of the training procedure through the `_train_impl` method.

        Args:
            *args: Variable length argument list to be passed to the training implementation.
            **kwargs: Arbitrary keyword arguments to be passed to the training implementation.

        Returns:
            None
        """

        self._train_impl()

    def save(self, checkpoint: str, *args, **kwargs) -> None:
        """
        Save the model checkpoint.
        This method saves the model to the specified checkpoint path.
        Args:
            checkpoint (str): Path where the model checkpoint will be saved.
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.
        Returns:
            None
        """

        self._save_model(checkpoint=checkpoint)

    @bind_hooks(before_stage=TrainingHookStages.ON_CLOSE)
    def close(self):
        """Clean up and exit."""
        self._save_model()
        self.agent.model.env.close()
        sys.exit(0)

    def _register_default_hooks(self) -> None:
        """Register default hooks common across implementations."""
        default_hooks = {
            TrainingHookStages.BEFORE_SETUP: [
                lambda _: print_base_model(self.config),
                lambda _: setup_debug_node(self.is_debug_mode),
                lambda _: setup_paths_dictionary(self, self.is_debug_mode),
                lambda _: self._write_config(),
            ],
            TrainingHookStages.AFTER_SETUP: [
                lambda _: self._set_resume_true(),
                lambda _: self.simulation_state_container.distribute(),
            ],
            TrainingHookStages.BEFORE_TRAINING: [lambda _: self._setup_monitoring()],
            TrainingHookStages.AFTER_TRAINING: [
                lambda _: rospy.on_shutdown(
                    lambda: self._save_model(checkpoint="last_model")
                )
            ],
        }

        for hook, callbacks in default_hooks.items():
            self.hook_manager.register(hook, callbacks)

    def _set_resume_true(self):
        """Set resume to True to make sure one can directly load and resume training."""
        self.config.resume = True

    def _write_config(self):
        """Write configuration to file if not in debug mode."""
        if not self.is_debug_mode:
            write_config_yaml(
                self.config.model_dump(),
                self.paths[Paths.Agent].path / "training_config.yaml",
            )

    @bind_hooks(before_stage=TrainingHookStages.ON_SAVE)
    def _save_model(self, checkpoint: str) -> None:
        """Save the trained model."""
        if not self.is_debug_mode:
            self.agent.model.save(
                dirpath=self.paths[Paths.Agent].path, file_name=checkpoint
            )

    def _register_framework_specific_hooks(self):
        """Register hooks that are specific to the framework."""
        pass

    def _train_impl(self, *args, **kwargs) -> None:
        """Implementation of training logic."""
        self.agent.model.train()
        self._save_model(checkpoint="last_model")

    @abstractmethod
    def _setup_agent(self, *args, **kwargs) -> None:
        """Initialize the RL agent."""
        raise NotImplementedError()

    @abstractmethod
    def _setup_environment(self, *args, **kwargs) -> None:
        """Setup training Gym environment."""
        raise NotImplementedError()

    @abstractmethod
    def _setup_monitoring(self, *args, **kwargs) -> None:
        """Setup monitoring tools."""
        raise NotImplementedError()

    def _setup_simulation_state_container(self, *args, **kwargs) -> None:
        """Initialize agent state container."""
        self.simulation_state_container = get_arena_states(
            goal_radius=self.config.arena_cfg.general.goal_radius,
            max_steps=self.config.arena_cfg.general.max_num_moves_per_eps,
            is_discrete=self.config.agent_cfg.action_space.is_discrete,
            safety_distance=self.config.arena_cfg.general.safety_distance,
            robot_cfg=self.config.arena_cfg.robot,
            task_modules_cfg=self.config.arena_cfg.task,
        )

    def _setup_agent_state_container(self, *args, **kwargs) -> None:
        """Initialize agent state container."""
        self.agent_state_container: rosnav_rl.AgentStateContainer = (
            self.simulation_state_container.to_agent_state_container()
        )

    @property
    def is_debug_mode(self):
        return rospy.get_param("debug_mode", False)

    @property
    def is_resume(self):
        return self.__resume
