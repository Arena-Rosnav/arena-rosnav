from launch import LaunchDescription
import launch
from launch_ros.actions import Node

def generate_launch_description():
    logger = launch.substitutions.LaunchConfiguration("log_level")
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
        ),
        Node(
            package='ros2isaacsim',
            executable='run_isaacsim',
            output='screen', 
            arguments=['--ros-args', '--log-level', logger]
        ),
    ])