from typing import Tuple

from rosnav_rl.observations import DONE_REASONS


def determine_termination(
    reward_info: dict,
    curr_steps: int,
    max_steps: int,
    info: dict = None,
) -> Tuple[dict, bool]:
    """
    Determine if the episode should terminate.

    Args:
        reward_info (dict): The reward information.
        curr_steps (int): The current number of steps in the episode.
        max_steps (int): The maximum number of steps per episode.
        info (dict): Additional information.

    Returns:
        tuple: A tuple containing the info dictionary and a boolean flag indicating if the episode should terminate.

    """

    if info is None:
        info = {}

    terminated = reward_info["is_done"]

    if terminated:
        info["done_reason"] = reward_info["done_reason"]
        info["is_success"] = reward_info["is_success"]
        info["episode_length"] = curr_steps

    if curr_steps >= max_steps:
        terminated = True
        info["done_reason"] = DONE_REASONS.STEP_LIMIT
        info["is_success"] = 0
        info["episode_length"] = curr_steps

    return info, terminated
