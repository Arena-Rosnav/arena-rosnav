
import launch


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='namespace'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
