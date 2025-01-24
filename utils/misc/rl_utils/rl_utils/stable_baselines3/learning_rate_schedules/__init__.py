from typing import Callable

from .linear import linear_decay
from .square_root import square_root_decay


def load_lr_schedule(type: str, settings: dict) -> Callable:
    if type == "linear":
        return linear_decay(**settings)
    elif type == "square_root":
        return square_root_decay(**settings)
    else:
        raise NotImplementedError(f"Learning rate schedule '{type}' not implemented!")
