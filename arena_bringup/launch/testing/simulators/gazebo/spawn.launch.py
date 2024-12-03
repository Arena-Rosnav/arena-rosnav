
import launch
import launch_ros


class LaunchArgument(launch.actions.DeclareLaunchArgument):
    @property
    def substitution(self):
        return launch.substitutions.LaunchConfiguration(self.name)

    @property
    def parameter(self):
        return {self.name: self.substitution}


def generate_launch_description():
    # Set environment variables
    name = LaunchArgument(
        name='name'
    )

    description = LaunchArgument(
        name='description'
    )

    # # Spawn the robot into the Gazebo simulation
    spawn_model = launch_ros.actions.Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        # arguments=[
        #     '-world', 'default',
        #     '-string', robot_description,
        #     '-name', robot_model,
        #     '-allow_renaming', 'false',
        #     '-x', '0',
        #     '-y', '0',
        #     '-z', '0',
        # ],
        parameters=[
            {
                "world": "default",
                "string": description.substitution,
                "name": name.substitution,
                "allow_renaming": False,
                "topic": 'robot_description',
            }
        ],
    )

    # Return the LaunchDescription with all the nodes/actions
    return launch.LaunchDescription(
        [
            name,
            description,
            spawn_model,
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
