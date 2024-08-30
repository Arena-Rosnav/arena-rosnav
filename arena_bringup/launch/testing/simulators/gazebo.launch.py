import os
import subprocess

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Find Gazebo in the build directory ---
    gazebo_path = os.path.join(get_package_share_directory('ros_gz_sim'), 'lib', 'gazebo')
    gz_sim_path = os.path.join(gazebo_path, 'gz')

    # Set environment variables
    env = os.environ.copy()
    env['GZ_CONFIG_PATH'] = os.path.join(gazebo_path, 'share', 'gz')
    env['PATH'] = f"{gz_sim_path}:{env['PATH']}"

    # Launch Arguments
    ld = launch.LaunchDescription([

        # Launch gz_sim (Gazebo)
        launch.actions.ExecuteProcess(
            cmd=['ruby', os.path.join(gz_sim_path, 'gz'), 'sim', '--force-version', '7',
                 launch.substitutions.LaunchConfiguration('world_file')],
            env=env,
            output='screen'
        ),
    
        # # Launch robot model
        # launch.actions.IncludeLaunchDescription(
        #     launch.launch_description_sources.PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory(
        #             'ros_gz_sim'), 'launch', 'gz_spawn_entity.launch.py')
        #     ),
        #     launch_arguments={
        #         'world_name': launch.substitutions.LaunchConfiguration('world_file'),
        #         'name': launch.substitutions.LaunchConfiguration('model'),
        #         'sdf': launch.substitutions.LaunchConfiguration('robot_setup_file')
        #     }.items()
        # ),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()