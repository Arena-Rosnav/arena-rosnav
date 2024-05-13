from typing import Callable


def square_root_decay(
    initial_value: float, final_value: float
) -> Callable[[float], float]:
    """
    Square root decay schedule.

    Args:
        initial_value (float): Initial value.
        final_value (float): Final value.

    Returns:
        Callable[[float], float]: Schedule that computes current value depending on remaining progress.
    """
    return (
        lambda progress_remaining: initial_value
        - (initial_value - final_value) * (1 - progress_remaining) ** 0.5
    )
