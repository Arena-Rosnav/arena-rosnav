from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from arena_bringup.substitutions import LaunchArgument


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchArgument('use_sim_time', default_value='true')
    namespace = LaunchArgument('namespace')

    return LaunchDescription([
        use_sim_time,
        namespace,

        # Agent Manager Node
        Node(
            package='hunav_agent_manager',
            executable='hunav_agent_manager',
            namespace=namespace.substitution,
            name='hunav_agent_manager',
            output='screen',
            parameters=[
                use_sim_time.param(bool)
            ]
        )
    ])
