
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

from arena_bringup.substitutions import LaunchArgument


def generate_launch_description():

    ss_path = get_package_share_directory('simulation-setup')

    use_sim_time = LaunchArgument("use_sim_time")

    namespace = LaunchArgument("namespace")
    robot = LaunchArgument("robot")
    frame = LaunchArgument("frame")

    global_planner = LaunchArgument("global_planner")
    local_planner = LaunchArgument("local_planner")

    # Include the Nav2 launch file
    nav2_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [
                    ss_path,
                    "launch",
                    "nav2.launch.py",
                ]
            )),
        launch_arguments={
            "use_sim_time": use_sim_time.substitution,
            "robot": robot.substitution,
            "namespace": namespace.substitution,
            "global_planner": global_planner.substitution,
            "local_planner": local_planner.substitution,
            "frame": frame.substitution,
        }.items(),
    )

    # launch robot control
    control_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [
                    ss_path,
                    "entities",
                    "robots",
                    robot.substitution,
                    "launch",
                    "control.launch.py",
                ]
            )),
        launch_arguments={
            "use_sim_time": use_sim_time.substitution,
            "frame": frame.substitution,
            "namespace": namespace.substitution,
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
                "use_sim_time": use_sim_time.substitution,
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
                "imu0": "/imu0",
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
            namespace=namespace.substitution,
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            parameters=[use_sim_time.parameter],
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom_to_base_link_publisher",
            namespace=namespace.substitution,
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            parameters=[use_sim_time.parameter],
        ),
    ]

    ld = launch.LaunchDescription([
        use_sim_time,
        global_planner,
        local_planner,
        robot,
        namespace,
        frame,
        launch.actions.DeclareLaunchArgument(
            name='train_mode',
            default_value='false',
            description='If false, start the Rosnav Deployment launch.actions.Nodes'
        ),
        launch.actions.DeclareLaunchArgument(
            name='agent_name',
            default_value='',
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
        control_launch,
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
