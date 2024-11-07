from enum import Enum, auto
from functools import wraps
from typing import Callable, TypeVar, Any, Dict, List, TYPE_CHECKING
from typing_extensions import ParamSpec

P = ParamSpec("P")
T = TypeVar("T")

if TYPE_CHECKING:
    from rl_utils.trainer.arena_trainer import ArenaTrainer


class TrainingHookStages(Enum):
    """Training pipeline hook stages.

    ON_INIT
        -> self.__init__()

    BEFORE_SETUP
        -> self._setup()
    AFTER_SETUP

    BEFORE_TRAINING
        -> self.train()
    AFTER_TRAINING

    ON_SAVE
        self._save_model()

    ON_CLOSE
        self.close()
    """

    ON_INIT = auto()
    BEFORE_SETUP = auto()
    AFTER_SETUP = auto()
    BEFORE_TRAINING = auto()
    AFTER_TRAINING = auto()
    ON_SAVE = auto()
    ON_CLOSE = auto()


class HookManager:
    """Manages hook registration and execution."""

    def __init__(self):
        self.__hooks: Dict[TrainingHookStages, List[Callable]] = {
            stage: [] for stage in TrainingHookStages
        }

    def register(self, stage: TrainingHookStages, callback: Callable) -> None:
        """Register a callback for a specific hook stage."""
        if type(stage) is not TrainingHookStages:
            raise TypeError(
                f"Invalid stage type: {type(stage)} is not a 'TrainingHookStages'!"
            )

        if isinstance(callback, list):
            for cb in callback:
                self.__hooks[stage].append(cb)
        else:
            self.__hooks[stage].append(callback)

    def run(self, stage: TrainingHookStages, context: Any = None) -> None:
        """Execute all callbacks for a given stage."""
        for callback in self.__hooks[stage]:
            callback(context)


def bind_hooks(
    before_stage: TrainingHookStages = None, after_stage: TrainingHookStages = None
):
    """Decorator to run hooks before and/or after method execution."""

    def decorator(func: Callable[P, T]) -> Callable[P, T]:
        @wraps(func)
        def wrapper(self: "ArenaTrainer", *args: P.args, **kwargs: P.kwargs) -> T:
            if before_stage:
                self.hook_manager.run(before_stage, self)

            result = func(self, *args, **kwargs)

            if after_stage:
                self.hook_manager.run(after_stage, self)

            return result

        return wrapper

    return decorator
