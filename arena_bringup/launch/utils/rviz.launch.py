
import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='show_rviz',
            default_value='true',
            description='Wether to show rviz or not'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            parameters=[
                {
                    "use_sim_time": launch.substitutions.LaunchConfiguration('use_sim_time')
                }
            ],
            output="screen",
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
