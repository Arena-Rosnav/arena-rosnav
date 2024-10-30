from typing import TypedDict


class TrainArguments(TypedDict):
    total_timesteps: int
    callback: callable
    progress_bar: bool = False
    log_interval: int = 1
    tb_log_name: str = "PPO"
    reset_num_timesteps: bool = False
