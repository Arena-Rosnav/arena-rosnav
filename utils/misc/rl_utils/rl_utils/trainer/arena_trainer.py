import sys
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

import rl_utils.utils.paths as Paths
import rospy

if TYPE_CHECKING:
    from rl_utils.cfg.train import TrainingCfg
from rosnav_rl.states import SimulationStateContainer
from rl_utils.utils.hooks import HookManager, TrainingHookStages, bind_hooks
from rosnav_rl.utils.type_aliases import SupportedRLFrameworks
from rl_utils.utils.type_alias.observation import EnvironmentType, PathsDict
from rosnav_rl.rl_agent import RL_Agent
from tools.config import ConfigManager
from tools.general import (
    print_base_model,
    setup_debug_node,
    setup_paths_dictionary,
    write_config_yaml,
)


@dataclass
class TrainingArguments:
    """Arguments for training."""

    def to_dict(self) -> dict:
        return self.__dict__


class ArenaTrainer(ABC):
    """
    ArenaTrainer is an abstract base class for setting up and managing the training process of a reinforcement learning
    (RL) agent within a simulation environment. It provides a structured way to configure, train, and save RL models,
    as well as to register and manage hooks for various stages of the training lifecycle.

    Attributes:
        FRAMEWORK (RLFramework): The training framework being used.
        config (TrainingCfg): Configuration settings for the training process.
        paths (PathsDict): Dictionary containing paths for saving models and configurations.
        simulation_state_container (SimulationStateContainer): Container for storing the simulation state.
        agent (RL_Agent): The RL agent from Rosnav-RL being trained.
        environment (EnvironmentType): The simulation environment.
        hook_manager (HookManager): Manages hooks for different stages of the training process.
    """

    __framework: SupportedRLFrameworks

    config: "TrainingCfg"
    paths: PathsDict

    simulation_state_container: SimulationStateContainer
    agent: RL_Agent
    environment: EnvironmentType

    config_manager: ConfigManager
    hook_manager: HookManager = HookManager()

    @bind_hooks(before_stage=TrainingHookStages.ON_INIT)
    def __init__(self, config: "TrainingCfg", resume: bool = False) -> None:
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
        Sets up the trainer by executing a series of predefined setup steps.

        The setup steps include:
        - Setting up the simulation state container
        - Setting up the agent
        - Setting up paths dictionary
        - Setting up the environment
        - Setting up training method arguments
        - Setting up monitoring

        This method ensures that all necessary components are initialized and ready for training.
        """
        setup_steps = [
            self._setup_agent_state_container,
            self._setup_agent,
            self._setup_environment,
            self._setup_monitoring,
        ]

        for step in setup_steps:
            step()

    @bind_hooks(
        before_stage=TrainingHookStages.BEFORE_TRAINING,
        after_stage=TrainingHookStages.AFTER_TRAINING,
    )
    def train(self, *args, **kwargs) -> None:
        """
        Train the model using the provided training implementation.

        Returns:
            None
        """
        self._train_impl()

    def save(self, checkpoint: str, *args, **kwargs) -> None:
        """
        Save the trained model.

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
            ],
            TrainingHookStages.AFTER_SETUP: [
                lambda _: self._set_resume_true(),
                lambda _: self._write_config(),
                lambda _: self.simulation_state_container.distribute(),
            ],
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
                dirpath=self.paths[Paths.Agent].path, checkpoint_name=checkpoint
            )

    def _register_framework_specific_hooks(self):
        """Register hooks that are specific to the framework."""
        pass

    def _train_impl(self, *args, **kwargs) -> None:
        """Implementation of training logic."""
        self.agent.model.train()

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

    @abstractmethod
    def _setup_agent_state_container(self, *args, **kwargs) -> None:
        """Initialize agent state container."""
        raise NotImplementedError()

    @property
    def is_debug_mode(self):
        return rospy.get_param("debug_mode", False)

    @property
    def is_resume(self):
        return self.__resume
