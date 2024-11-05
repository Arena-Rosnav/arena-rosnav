import sys
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

import gymnasium
import rl_utils.utils.paths as Paths
import rospy
from rl_utils.cfg.train import TrainingCfg
from rl_utils.state_container import SimulationStateContainer
from rl_utils.utils.hooks import HookManager, TrainingHookStages, bind_hooks
from rl_utils.utils.paths import PathFactory
from rl_utils.utils.type_alias.observation import PathsDict
from rosnav_rl.rl_agent import RL_Agent
from tools.config import ConfigManager
from tools.general import (
    print_base_model,
    save_model,
    write_config_yaml,
)


@dataclass
class TrainingArguments:
    """Arguments for training."""

    def to_dict(self) -> dict:
        return self.__dict__


class TrainingFramework(Enum):
    SB3 = "StableBaselines3"


class ArenaTrainer(ABC):
    """
    ArenaTrainer is an abstract base class for setting up and managing the training process of a reinforcement learning
    (RL) agent within a simulation environment. It provides a structured way to configure, train, and save RL models,
    as well as to register and manage hooks for various stages of the training lifecycle.

    Attributes:
        FRAMEWORK (TrainingFramework): The training framework being used.
        config (TrainingCfg): Configuration settings for the training process.
        paths (PathsDict): Dictionary containing paths for saving models and configurations.
        simulation_state_container (SimulationStateContainer): Container for storing the simulation state.
        agent (RL_Agent): The RL agent from Rosnav-RL being trained.
        environment (gymnasium.Env): The simulation environment.
        hook_manager (HookManager): Manages hooks for different stages of the training process.
    """

    FRAMEWORK: TrainingFramework

    config: TrainingCfg
    paths: PathsDict

    simulation_state_container: SimulationStateContainer
    agent: RL_Agent
    environment: gymnasium.Env

    config_manager: ConfigManager
    hook_manager: HookManager = HookManager()

    def __init__(self, config: TrainingCfg):
        """
        Initializes the ArenaTrainer with the given configuration.

        Args:
            config (TrainingCfg): The configuration object for training.
        """
        self.config = config
        self._register_default_hooks()
        self._setup()

    @bind_hooks(
        before_stage=TrainingHookStages.BEFORE_SETUP,
        after_stage=TrainingHookStages.AFTER_SETUP,
    )
    def _setup(self) -> None:
        """
        Sets up the trainer by executing a series of predefined setup steps.

        The setup steps include:
        - Registering framework-specific hooks
        - Setting up monitoring
        - Setting up the simulation state container
        - Setting up the agent
        - Setting up paths dictionary
        - Setting up the environment
        - Setting up training method arguments

        This method ensures that all necessary components are initialized and ready for training.
        """
        setup_steps = [
            self._register_framework_specific_hooks,
            self._setup_monitoring,
            self._setup_simulation_state_container,
            self._setup_agent,
            self._setup_paths,
            self._setup_environment,
        ]

        for step in setup_steps:
            step()

    @bind_hooks(
        before_stage=TrainingHookStages.BEFORE_TRAINING,
        after_stage=TrainingHookStages.AFTER_TRAINING,
    )
    def train(self, method_args: TrainingArguments = None) -> None:
        """
        Train the model using the provided training arguments.

        Args:
            method_args (TrainingArguments): The arguments to use for training. If not provided,
                                             it will use the 'method_arguments' attribute of the instance.

        Returns:
            None
        """
        self._train_impl(method_args or getattr(self, "method_arguments", {}))

    @bind_hooks(before_stage=TrainingHookStages.ON_CLOSE)
    def close(self):
        """Clean up and exit."""
        self._save_model()
        self.agent.model.env.close()
        sys.exit(0)

    def _register_default_hooks(self) -> None:
        """Register default hooks common across implementations."""
        default_hooks = {
            TrainingHookStages.ON_INIT: [lambda _: print_base_model(self.config)],
            TrainingHookStages.BEFORE_SETUP: [lambda _: self._setup_node()],
            TrainingHookStages.AFTER_SETUP: [
                lambda _: self._write_config(),
                lambda _: self.simulation_state_container.distribute(),
            ],
            TrainingHookStages.AFTER_TRAINING: [
                lambda _: rospy.on_shutdown(lambda: self._save_model())
            ],
        }

        for hook, callbacks in default_hooks.items():
            self.hook_manager.register(hook, callbacks)

    def _setup_node(self):
        """Setup ROS node."""
        rospy.set_param("debug_mode", self.is_debug_mode)
        if self.is_debug_mode:
            rospy.init_node("debug_node", disable_signals=False)

    def _write_config(self):
        """Write configuration to file if not in debug mode."""
        if not self.is_debug_mode:
            write_config_yaml(
                self.config.model_dump(),
                self.paths[Paths.Agent].path / "training_config.yaml",
            )

    @bind_hooks(before_stage=TrainingHookStages.ON_SAVE)
    def _save_model(self) -> None:
        """Save the trained model."""
        save_model(
            rl_model=self.agent,
            dirpath=self.paths[Paths.Agent].path,
            checkpoint_name="last_model",
            is_debug_mode=self.is_debug_mode,
        )

    def _train_impl(self, method_args: TrainingArguments) -> None:
        """Implementation of training logic."""
        self.agent.model.train(**method_args)

    def _setup_paths(self, *args, **kwargs):
        """Setup necessary directories."""
        self.paths = PathFactory.get_paths(self.agent.name)
        if not self.is_debug_mode:
            self.paths.create_all()

    def _register_framework_specific_hooks(self):
        """Register hooks that are specific to the framework."""
        pass

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
    def _setup_simulation_state_container(self, *args, **kwargs) -> None:
        """Initialize simulation state container."""
        raise NotImplementedError()

    def _setup_train_method_arguments(self, *args, **kwargs):
        """Prepare arguments for training method."""
        self.method_arguments = {}

    @property
    def is_debug_mode(self):
        return rospy.get_param("debug_mode", False)
