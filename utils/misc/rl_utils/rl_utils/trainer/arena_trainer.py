import sys
import time
from abc import ABC, abstractmethod
from typing import TypedDict

import rospy
from rl_utils.state_container import SimulationStateContainer
from rl_utils.utils.type_alias.observation import PathsDict
from rosnav_rl.rl_agent import RL_Agent
from tools.general import save_model_and_exit


class ArenaTrainer(ABC):
    agent: RL_Agent
    paths: PathsDict
    method_arguments: TypedDict

    @abstractmethod
    def _initialize_agent(self, *args, **kwargs):
        pass

    def _setup_directories(self, *args, **kwargs):
        pass

    @abstractmethod
    def _setup_environment(self, *args, **kwargs):
        pass

    def _setup_monitoring(self, *args, **kwargs):
        pass

    @abstractmethod
    def _init_simulation_state_container(
        self, *args, **kwargs
    ) -> SimulationStateContainer:
        pass

    @abstractmethod
    def _prepare_method_arguments(self, *args, **kwargs) -> TypedDict:
        return TypedDict()

    def _train_impl(self, method_args: TypedDict, *args, **kwargs):
        rospy.on_shutdown(
            lambda: save_model_and_exit(
                rl_model=self.agent,
                dirpath=self.paths["model"],
                checkpoint_name="last_model",
            )
        )

        start = time.time()
        self.agent.model.train(**method_args)
        print(
            f"Time passed: {time.time()-start}s. \n Training script will be terminated.."
        )
        self.close()

    def train(
        self,
        method_args: TypedDict = None,
        *args,
        **kwargs,
    ) -> callable:
        self._train_impl(
            method_args=method_args or self.method_arguments, *args, **kwargs
        )

    def close(self):
        sys.exit(0)
