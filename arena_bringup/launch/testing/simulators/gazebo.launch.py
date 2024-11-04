import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.conditions import IfCondition

def generate_launch_description():
    # Set environment variables
    current_dir = os.path.abspath(__file__)
    workspace_root = current_dir
    while not workspace_root.endswith('arena4_ws'):
        workspace_root = os.path.dirname(workspace_root)
        
    if not workspace_root.endswith('arena4_ws'):
        raise ValueError("Could not find the 'arena4_ws' directory in the current path.")

    # Set paths for Gazebo, Physics Engine, and Resource
    GZ_CONFIG_PATH = os.path.join(workspace_root, 'install', 'gz-sim7', 'share', 'gz')
    GZ_SIM_PHYSICS_ENGINE_PATH = os.path.join(workspace_root, 'build', 'gz-physics6')
    GZ_SIM_RESOURCE_PATHS = [
        os.path.join(workspace_root, 'src', 'gazebo', 'gz-sim', 'test', 'worlds', 'models'),
        os.path.join(workspace_root, 'src', 'arena', 'simulation-setup', 'entities'),
        os.path.join(workspace_root, 'src', 'arena', 'simulation-setup', 'worlds'),
        os.path.join(workspace_root, 'src', 'arena', 'simulation-setup', 'gazebo_models'),
        os.path.join(workspace_root, 'src', 'arena', 'simulation-setup', 'gazebo_models', 'Cafe table', 'materials', 'textures'),
        os.path.join(workspace_root, 'src', 'deps')
    ]
    GZ_SIM_RESOURCE_PATHS_COMBINED = ':'.join(GZ_SIM_RESOURCE_PATHS)
    
    # Update environment variables
    os.environ['GZ_CONFIG_PATH'] = GZ_CONFIG_PATH
    os.environ['GZ_SIM_PHYSICS_ENGINE_PATH'] = GZ_SIM_PHYSICS_ENGINE_PATH
    os.environ['GZ_SIM_RESOURCE_PATH'] = GZ_SIM_RESOURCE_PATHS_COMBINED

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')
    robot_model = LaunchConfiguration('model')
    random_spawn_test = LaunchConfiguration('random_spawn_test')

    # Construct the full world file path
    world_file_path = PathJoinSubstitution([
        workspace_root,
        'src', 'arena', 'simulation-setup', 'worlds',
        world_file,
        'worlds',
        PythonExpression(['"', world_file, '.world"'])
    ])

    # Gazebo launch
    gz_sim_launch_file = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_file),
        launch_arguments={
            'gz_args': [world_file_path, ' -v 4', ' -r', ' --render-engine ogre'],
            'physics-engine': 'gz-physics-dartsim'
        }.items()
    )

    # Robot URDF (Xacro) description
    robot_desc_path = os.path.join(
        workspace_root,
        'src', 'arena', 'simulation-setup', 'entities', 'robots',
        'jackal', 'urdf',
        'jackal.urdf.xacro'
    )

    # Process the robot description file using xacro
    doc = xacro.process_file(robot_desc_path, mappings={'use_sim': 'true'})
    robot_description = doc.toprettyxml(indent='  ')

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
        ]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Spawn the robot into the Gazebo simulation
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description,
            '-name', robot_model,
            '-allow_renaming', 'false'
        ],
    )

    # Bridge configuration
    bridge_config = os.path.join(
        workspace_root,
        'src', 'arena', 'arena-rosnav', 'arena_bringup',
        'launch', 'testing', 'simulators',
        'gazebo_bridge.yaml'
    )
    
    world_state_converter = Node(
        package='ros_gz_bridge',
        executable='world_state_to_odom',
        name='world_state_to_odom',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_name': robot_model,
            'world_frame': world_file,
            'robot_frame': 'base_link',
            'odom_frame': 'odom',
        }]
    )

    # Bridge to connect Gazebo and ROS2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
        }],
        arguments=[
            # Service bridges with correct ROS 2 service type notation
            '/world/default/control@ros_gz_interfaces.srv.ControlWorld',
            '/world/default/create@ros_gz_interfaces.srv.SpawnEntity',
            '/world/default/remove@ros_gz_interfaces.srv.DeleteEntity',
            '/world/default/set_pose@ros_gz_interfaces.srv.SetEntityPose',
            
            # Additional debugging
            '--ros-args',
            '--log-level', 'debug'
        ],
        remappings=[
            ('/world/default/pose/info', '/tf_static'),
            ('/world/default/dynamic_pose/info', '/tf'),
            (f'/world/default/model/{robot_model}/joint_state', '/joint_states'),
            ('/model/${robot_model}/cmd_vel', '/cmd_vel'),
            ('/model/${robot_model}/odometry', '/odom'),
            ('/world/default/model/${robot_model}/imu', '/imu'),
            ('/scan', '/scan'),
            ('/map', '/map'),
            ('/clock', '/clock')
        ]
    )

    # RViz configuration path
    rviz_config_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    )
    
    # Launch RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Delay RViz launch to ensure other nodes start first
    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz]
    )

    # Path to the Nav2 parameters file
    nav2_params_file = PathJoinSubstitution([
        workspace_root,
        'src', 'arena', 'simulation-setup', 'entities', 'robots', robot_model, 'configs', 'nav2.yaml'
    ])
    
    
    nav2_map_file = PathJoinSubstitution([
        workspace_root,
        'src', 'arena', 'simulation-setup', 'worlds',
        world_file,
        'map', 'map.yaml'
    ])
    
    nav2_launch_file = PathJoinSubstitution([
        workspace_root,
        'src', 'arena', 'simulation-setup', 'entities', 'robots', robot_model, 'launch', 'nav2.launch.py'
    ])


    # Include the Nav2 launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'map': nav2_map_file,
        }.items()
    )
    
    slam_launch_file = PathJoinSubstitution([
        workspace_root,
        'src', 'arena', 'simulation-setup', 'entities', 'robots', robot_model, 'launch', 'slam.launch.py'
    ])
    
    slam_yaml_file = PathJoinSubstitution([
        workspace_root,
        'src', 'arena', 'simulation-setup', 'entities', 'robots', robot_model, 'configs', 'slam.yaml'
    ])
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'yaml_path': slam_yaml_file,
        }.items()
    )
    
# Robot Localization Node
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frequency': 30.0,
            'two_d_mode': True,
            'publish_tf': True,
            'publish_acceleration': False,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',
            'odom0': '/odom',
            'odom0_config': [True,  True,  False,
                           False, False, True,
                           False, False, False,
                           False, False, True,
                           False, False, False],
            'imu0': '/imu',
            'imu0_config': [False, False, False,
                          True,  True,  True,
                          False, False, False,
                          True,  True,  True,
                          False, False, False],
            'publish_acceleration': False,
            'odom0_relative': False,  # Changed from false to False
            'imu0_relative': False,   # Changed from false to False
        }]
    )
    
    static_transform_publisher = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ]
    
    random_spawn_launch_file = PathJoinSubstitution([
        workspace_root,
        'src', 'arena', 'arena-rosnav', 'arena_bringup', 'launch', 'testing', 'simulators', 'gazebo_entity_spawn.py'
    ])

    random_spawn_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(random_spawn_launch_file)
    )

    # Return the LaunchDescription with all the nodes/actions
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('world_file', default_value='map_empty', description='World file name'),
        DeclareLaunchArgument('model', default_value='jackal', description='Robot model name'),
        SetEnvironmentVariable('GZ_CONFIG_PATH', GZ_CONFIG_PATH),
        SetEnvironmentVariable('GZ_SIM_PHYSICS_ENGINE_PATH', GZ_SIM_PHYSICS_ENGINE_PATH),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', GZ_SIM_RESOURCE_PATHS_COMBINED),
        *static_transform_publisher,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        robot_localization_node,
        spawn_robot,
        bridge,
        delayed_rviz,
        nav2_launch,
        slam_launch,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(random_spawn_launch_file),
            condition=IfCondition(random_spawn_test)
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()