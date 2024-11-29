import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.conditions import IfCondition


def generate_launch_description():
    # Set environment variables
    current_dir = os.path.abspath(__file__)
    workspace_root = current_dir
    while not workspace_root.endswith("arena4_ws"):
        workspace_root = os.path.dirname(workspace_root)
    if not workspace_root.endswith("arena4_ws"):
        raise ValueError(
            "Could not find the 'arena4_ws' directory in the current path."
        )

    # Set paths for Gazebo, Physics Engine, and Resource
    GZ_CONFIG_PATHS = [
        os.path.join(workspace_root, "install", "gz-sim8", "share", "gz"),
        # os.path.join(workspace_root, 'install', 'gz-tools2', 'share', 'gz'),
    ]

    GZ_SIM_PHYSICS_ENGINE_PATH = os.path.join(
        workspace_root, "build", "gz-physics7"
    )

    GZ_SIM_RESOURCE_PATHS = [
        os.path.join(workspace_root, "src", "deps", "robots", "jackal"),
        os.path.join(
            workspace_root, "src", "arena", "simulation-setup", "entities"
        ),
        os.path.join(
            workspace_root, "src", "arena", "simulation-setup", "worlds"
        ),
        os.path.join(
            workspace_root, "src", "arena", "simulation-setup", "gazebo_models"
        ),
        os.path.join(
            workspace_root, "src", "arena", "simulation-setup", "entities", "obstacles", "static"
        ),
        os.path.join(
            workspace_root,
            "src",
            "arena",
            "simulation-setup",
            "gazebo_models",
            "Cafe table",
            "materials",
            "textures",
        ),
    ]
    # GZ_CONFIG_PATH = ":".join(GZ_CONFIG_PATHS)
    GZ_CONFIG_PATH = "/usr/share/gz"

    GZ_SIM_RESOURCE_PATHS_COMBINED = ":".join(GZ_SIM_RESOURCE_PATHS)

    # Update environment variables
    # os.environ['GZ_CONFIG_PATH'] = GZ_CONFIG_PATH
    os.environ["GZ_CONFIG_PATH"] = GZ_CONFIG_PATH
    os.environ["GZ_SIM_PHYSICS_ENGINE_PATH"] = GZ_SIM_PHYSICS_ENGINE_PATH
    os.environ["GZ_SIM_RESOURCE_PATH"] = GZ_SIM_RESOURCE_PATHS_COMBINED

    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_file = LaunchConfiguration("world_file")
    robot_model = LaunchConfiguration("model")
    random_spawn_test = LaunchConfiguration("random_spawn_test")

    # Construct the full world file path
    world_file_path = PathJoinSubstitution(
        [
            workspace_root,
            "src",
            "arena",
            "simulation-setup",
            "worlds",
            world_file,
            "worlds",
            PythonExpression(['"', world_file, '.world"']),
        ]
    )

    # Gazebo launch
    gz_sim_launch_file = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_file),
        launch_arguments={
            "gz_args": [
                world_file_path,
                " -v 4",
                " -r",
                " --render-engine ogre",
            ],
            "physics-engine": "gz-physics-dartsim",
        }.items(),
    )

    # # Robot URDF (Xacro) description
    # robot_desc_path = os.path.join(
    #     workspace_root,
    #     "src",
    #     "arena",
    #     "simulation-setup",
    #     "entities",
    #     "robots",
    #     "jackal",
    #     "urdf",
    #     "jackal.urdf.xacro",
    # )

    # # Process the robot description file using xacro
    # # doc = xacro.process_file(robot_desc_path, mappings={'use_sim': 'true'})
    # # robot_description = doc.toprettyxml(indent='  ')
    # # Process the robot description file using xacro
    # robot_description = xacro.process_file(
    #     robot_desc_path, mappings={"use_sim": "true"}
    # ).toxml()

    # # Robot State Publisher
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="both",
    #     parameters=[
    #         {"use_sim_time": use_sim_time},
    #         {"robot_description": robot_description},
    #     ],
    # )

    # # Joint State Publisher
    # joint_state_publisher = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     name="joint_state_publisher",
    #     parameters=[{"use_sim_time": use_sim_time}],
    # )

    # # Spawn the robot into the Gazebo simulation
    # spawn_robot = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     output="screen",
    #     # arguments=[
    #     #     '-world', 'default',
    #     #     '-string', robot_description,
    #     #     '-name', robot_model,
    #     #     '-allow_renaming', 'false',
    #     #     '-x', '0',
    #     #     '-y', '0',
    #     #     '-z', '0',
    #     # ],
    #     parameters=[
    #         {
    #             "world": "default",
    #             "string": robot_description,
    #             "name": robot_model,
    #             "allow_renaming": False,
    #             "topic": 'robot_description',
    #         }
    #     ],
    # )

    # Bridge configuration
    bridge_config = os.path.join(
        workspace_root,
        "src",
        "arena",
        "arena-rosnav",
        "arena_bringup",
        "launch",
        "testing",
        "simulators",
        "gazebo_bridge.yaml",
    )

    world_state_converter = Node(
        package="ros_gz_bridge",
        executable="world_state_to_odom",
        name="world_state_to_odom",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_name": robot_model,
                "world_frame": world_file,
                "robot_frame": "base_link",
                "odom_frame": "odom",
            }
        ],
    )

    gz_topic = '/model/robot'
    joint_state_gz_topic = '/world/default' + gz_topic + '/joint_state'
    link_pose_gz_topic = gz_topic + '/pose'

    # Bridge to connect Gazebo and ROS2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            # Existing clock bridge
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (Gazebo -> ROS2)
            joint_state_gz_topic + '@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Link poses (Gazebo -> ROS2)
            link_pose_gz_topic + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            link_pose_gz_topic + \
                '_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Velocity and odometry (Gazebo -> ROS2)
            gz_topic + '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            gz_topic + '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            (joint_state_gz_topic, 'joint_states'),
            (link_pose_gz_topic, '/tf'),
            (link_pose_gz_topic + '_static', '/tf_static'),
        ],
        parameters=[
            {
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
                'use_sim_time': use_sim_time
            }
        ],
    )

    # RViz configuration path
    rviz_config_file = os.path.join(
        workspace_root,
        "src",
        "deps",
        "navigation2",
        "nav2_bringup",
        "rviz",
        "nav2_default_view.rviz",
    )

    # Launch RViz node
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    random_spawn_launch_file = PathJoinSubstitution(
        [
            workspace_root,
            "src",
            "arena",
            "arena-rosnav",
            "arena_bringup",
            "launch",
            "testing",
            "simulators",
            "gazebo_entity_spawn.py",
        ]
    )

    random_spawn_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(random_spawn_launch_file)
    )

    # Delay RViz launch to ensure other nodes start first
    delayed_rviz = TimerAction(period=5.0, actions=[rviz])

    # Return the LaunchDescription with all the nodes/actions
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value="map_empty",
                description="World file name",
            ),
            DeclareLaunchArgument(
                "model", default_value="jackal", description="Robot model name"
            ),
            SetEnvironmentVariable("GZ_CONFIG_PATH", GZ_CONFIG_PATH),
            SetEnvironmentVariable(
                "GZ_SIM_PHYSICS_ENGINE_PATH", GZ_SIM_PHYSICS_ENGINE_PATH
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", GZ_SIM_RESOURCE_PATHS_COMBINED
            ),
            gazebo,
            # robot_state_publisher,
            # joint_state_publisher,
            # spawn_robot,
            bridge,
            delayed_rviz,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(random_spawn_launch_file),
                condition=IfCondition(random_spawn_test),
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
