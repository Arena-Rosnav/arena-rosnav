from typing import Callable
from typing import Callable


def linear_decay(initial_value: float, final_value: float) -> Callable[[float], float]:
    """
    Linear decay schedule.

    Args:
        initial_value (float): Initial value.
        final_value (float): Final value.
        decay_steps (int): Number of steps to decay over.

    Returns:
        Callable[[float], float]: Schedule that computes current value depending on remaining progress.
    """

    def func(progress_remaining: float) -> float:
        """
        Compute the current value.

        Args:
            progress_remaining (float): Remaining progress.

        Returns:
            float: Current value.
        """
        return initial_value - (initial_value - final_value) * (1 - progress_remaining)

    return func
