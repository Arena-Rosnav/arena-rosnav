import sys
import time
from abc import ABC, abstractmethod
from enum import Enum
from typing import Callable, Dict, List, TypedDict, Union

import gymnasium
from rl_utils.utils.paths import PathFactory
import rospy
from pydantic import BaseModel
from pydantic.dataclasses import dataclass
from rl_utils.cfg.train import TrainingCfg
from rl_utils.state_container import SimulationStateContainer
from rl_utils.utils.type_alias.observation import PathsDict
from rosnav_rl.rl_agent import RL_Agent
from tools.general import (
    create_directories,
    print_base_model,
    save_model,
    write_config_yaml,
)

import rl_utils.utils.paths as Paths


@dataclass
class Timer:
    start_time: float = 0
    duration: float = 0

    def start(self):
        if self.start_time != 0:
            raise ValueError("Timer already started.")
        self.start_time = time.time()

    def stop(self):
        if self.start_time == 0:
            raise ValueError("Timer not started.")
        self.duration = time.time() - self.start_time


class TrainingFramework(Enum):
    SB3 = "StableBaselines3"


class TrainerHook(Enum):
    ON_INIT = "on_init"
    BEFORE_SETUP = "before_setup"
    AFTER_SETUP = "after_setup"
    BEFORE_TRAINING = "before_training"
    AFTER_TRAINING = "after_training"
    ON_SAVE = "on_save"
    ON_CLOSE = "on_close"


class ArenaTrainer(ABC):
    FRAMEWORK: TrainingFramework

    config: TrainingCfg
    paths: PathsDict

    simulation_state_container: SimulationStateContainer
    agent: RL_Agent
    environment: gymnasium.Env

    __hooks: Dict[TrainerHook, List[Callable]]

    def __init__(self, config: TrainingCfg):
        self.config = config
        self.timer = Timer()
        self.__hooks: Dict[TrainerHook, List[Callable]] = {
            stage: [] for stage in TrainerHook
        }
        self._initialize_trainer()

    def _initialize_trainer(self):
        """Initialize the trainer with default hooks and configuration."""
        self._register_default_hooks()
        self._run_hooks(TrainerHook.ON_INIT)

    def register_hook(
        self, hook_name: Union[TrainerHook, str], callback: Callable
    ) -> None:
        """Register a new hook callback."""
        if not isinstance(hook_name, TrainerHook):
            hook_name = TrainerHook(hook_name)
        self.__hooks.setdefault(hook_name, []).append(callback)

    def setup(self) -> None:
        """Set up the training environment and components."""
        self._run_hooks(TrainerHook.BEFORE_SETUP)
        self._perform_setup()
        self._run_hooks(TrainerHook.AFTER_SETUP)

    def train(self, method_args: TypedDict = None) -> None:
        """Execute the training process with error handling and metrics tracking."""
        self._run_hooks(TrainerHook.BEFORE_TRAINING)
        self.timer.start()

        try:
            self._train_impl(method_args or getattr(self, "method_arguments", {}))
        finally:
            self.timer.stop()
            self._log_training_completion()
            self._run_hooks(TrainerHook.AFTER_TRAINING)

    def close(self):
        """Clean up and exit."""
        self._run_hooks(TrainerHook.ON_CLOSE)

    def _perform_setup(self) -> None:
        """Execute all setup steps in sequence."""
        setup_steps = [
            self._register_framework_specific_hooks,
            self._setup_monitoring,
            self._setup_simulation_state_container,
            self._setup_agent,
            self._setup_paths,
            self._setup_environment,
            self._setup_train_method_arguments,
        ]
        for step in setup_steps:
            step()

    def _register_default_hooks(self) -> None:
        """Register default hooks common across implementations."""
        default_hooks = {
            TrainerHook.ON_INIT: [lambda _: print_base_model(self.config)],
            TrainerHook.BEFORE_SETUP: [lambda _: self._setup_node(self.is_debug_mode)],
            TrainerHook.AFTER_SETUP: [lambda _: self._write_config(self.is_debug_mode)],
            TrainerHook.AFTER_TRAINING: [
                lambda _: rospy.on_shutdown(
                    lambda: self._run_hooks(TrainerHook.ON_SAVE)
                )
            ],
            TrainerHook.ON_SAVE: [lambda _: self._save_model()],
            TrainerHook.ON_CLOSE: [
                lambda _: self.agent.model.env.close(),
                lambda _: sys.exit(0),
            ],
        }

        for hook, callbacks in default_hooks.items():
            for callback in callbacks:
                self.register_hook(hook, callback)

    def _run_hooks(self, hook_name: Union[TrainerHook, str]):
        """Execute all callbacks registered for a specific hook."""
        for callback in self.__hooks.get(hook_name, [lambda: None]):
            callback(self)

    def _setup_node(self, debug_mode: bool = False):
        """Setup ROS node."""
        rospy.set_param("debug_mode", debug_mode)
        if debug_mode:
            rospy.init_node("debug_node", disable_signals=False)

    def _write_config(self, debug_mode: bool = False):
        """Write configuration to file if not in debug mode."""
        if not debug_mode:
            write_config_yaml(
                self.config.model_dump(),
                self.paths[Paths.Agent].path / "training_config.yaml",
            )

    def _setup_paths(self, *args, **kwargs):
        """Setup necessary directories."""
        self.paths: PathsDict = PathFactory.get_paths(self.agent.name)
        if not self.is_debug_mode:
            for path in self.paths.values():
                path.create()

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
    def _setup_monitoring(self, *args, **kwargs):
        """Setup monitoring tools."""
        raise NotImplementedError()

    @abstractmethod
    def _setup_simulation_state_container(self, *args, **kwargs):
        """Initialize simulation state container."""
        raise NotImplementedError()

    def _setup_train_method_arguments(self, *args, **kwargs):
        """Prepare arguments for training method."""
        self.method_arguments = {}

    def _train_impl(self, method_args: TypedDict):
        """Implementation of training logic."""
        self.agent.model.train(**method_args)

    def _save_model(self) -> None:
        """Save the trained model."""
        save_model(
            rl_model=self.agent,
            dirpath=self.paths[Paths.Agent].path,
            checkpoint_name="last_model",
            is_debug_mode=self.is_debug_mode,
        )

    def _log_training_completion(self) -> None:
        """Log training completion message."""
        print(
            f"Time passed: {self.timer.duration}s. \nTraining script will be terminated.."
        )

    @property
    def is_debug_mode(self):
        return rospy.get_param("debug_mode", False)
