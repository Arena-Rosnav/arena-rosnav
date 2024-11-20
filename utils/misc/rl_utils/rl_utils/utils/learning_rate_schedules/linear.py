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


def linear_schedule(initial_value: float) -> Callable[[float], float]:
    """
    Linear learning rate schedule.

    Args:
        initial_value (float): Initial learning rate.

    Returns:
        Callable[[float], float]: Schedule that computes current learning rate depending on remaining progress.
    """

    def func(progress_remaining: float) -> float:
        """
        Compute the current learning rate.

        Args:
            progress_remaining (float): Remaining progress.

        Returns:
            float: Current learning rate.
        """
        return progress_remaining * initial_value

    return func
