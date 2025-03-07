
import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='v_lin',
            default_value='0.3'
        ),
        launch.actions.DeclareLaunchArgument(
            name='v_ang',
            default_value='0.1'
        ),
        launch.actions.DeclareLaunchArgument(
            name='usm',
            default_value='true'
        ),
        launch_ros.actions.Node(
            package='sensor_simulator',
            executable='sensorsim_node_tmgr.py',
            name='sensorsim_node',
            output='screen'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
