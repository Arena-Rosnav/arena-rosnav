import os


def set_dynamic_reconfigure_parameter(node: str, param: str, value: str) -> None:
    """Set curriculum parameters via ROS dynamic reconfigure."""
    os.system(f"rosrun dynamic_reconfigure dynparam set /{node} {param} {value}")
