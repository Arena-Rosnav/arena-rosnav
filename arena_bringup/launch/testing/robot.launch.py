
import launch
import launch_ros


def generate_launch_description():

    use_sim_time = launch.substitutions.LaunchConfiguration("use_sim_time")
    world_path = launch.substitutions.LaunchConfiguration("world_path")
    namespace = launch.substitutions.LaunchConfiguration("namespace")
    robot_path = launch.substitutions.LaunchConfiguration("robot_path")

    # Path to the Nav2 parameters file
    nav2_params_file = launch.substitutions.PathJoinSubstitution(
        [
            robot_path,
            "configs",
            "nav2.yaml",
        ]
    )

    nav2_map_file = launch.substitutions.PathJoinSubstitution(
        [
            world_path,
            "map",
            "map.yaml",
        ]
    )

    nav2_launch_file = launch.substitutions.PathJoinSubstitution(
        [
            robot_path,
            "launch",
            "nav2.launch.py",
        ]
    )

    # Include the Nav2 launch file
    nav2_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            nav2_launch_file),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            "map": nav2_map_file,
        }.items(),
    )

    slam_launch_file = launch.substitutions.PathJoinSubstitution(
        [
            robot_path,
            "launch",
            "slam.launch.py",
        ]
    )

    slam_yaml_file = launch.substitutions.PathJoinSubstitution(
        [
            robot_path,
            "configs",
            "slam.yaml",
        ]
    )

    slam_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            slam_launch_file),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "yaml_path": slam_yaml_file,
        }.items(),
    )

    # Robot Localization launch.actions.Node
    robot_localization_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "frequency": 30.0,
                "two_d_mode": True,
                "publish_tf": True,
                "publish_acceleration": False,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_link_frame": "base_link",
                "world_frame": "odom",
                "odom0": "/odom",
                "odom0_config": [
                    True,
                    True,
                    False,
                    False,
                    False,
                    True,
                    False,
                    False,
                    False,
                    False,
                    False,
                    True,
                    False,
                    False,
                    False,
                ],
                "imu0": "/imu",
                "imu0_config": [
                    False,
                    False,
                    False,
                    True,
                    True,
                    True,
                    False,
                    False,
                    False,
                    True,
                    True,
                    True,
                    False,
                    False,
                    False,
                ],
                "publish_acceleration": False,
                "odom0_relative": False,  # Changed from false to False
                "imu0_relative": False,  # Changed from false to False
            }
        ],
    )

    static_transform_publisher = [
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_publisher",
            namespace=namespace,
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom_to_base_link_publisher",
            namespace=namespace,
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "world_path",
            default_value="map_empty",
            description="World file name",
        ),
        launch.actions.DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock if true",
        ),
        launch.actions.DeclareLaunchArgument(
            name='inter_planner'
        ),
        launch.actions.DeclareLaunchArgument(
            name='local_planner',
            default_value='',
            description='local planner type [teb, dwa, mpc, rlca, arena, rosnav]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='name',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='robot_path',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='namespace',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='sim_namespace',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='frame',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='simulator',
            default_value='flatland'
        ),
        launch.actions.DeclareLaunchArgument(
            name='train_mode',
            default_value='false',
            description='If false, start the Rosnav Deployment launch.actions.Nodes'
        ),
        launch.actions.DeclareLaunchArgument(
            name='agent_name',
            default_value=launch.substitutions.LaunchConfiguration('name'),
            description='DRL agent name to be deployed'
        ),
        launch.actions.DeclareLaunchArgument(
            name='complexity',
            default_value='1'
        ),
        launch.actions.DeclareLaunchArgument(
            name='record_data',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='record_data_dir',
            default_value='auto:'
        ),
        *static_transform_publisher,
        robot_localization_node,
        nav2_launch,
        # slam_launch,
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
