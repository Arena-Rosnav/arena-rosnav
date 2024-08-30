import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Set environment variables
    gz_config_path = "/root/arena4_ws/install/gz-sim7/share/gz"  # Set the path directly
    env = os.environ.copy()
    env['GZ_CONFIG_PATH'] = gz_config_path
    
    # Get the path to the gz_sim.launch.py file
    gz_sim_launch_file = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    # Launch gz_sim.launch.py as a subprocess
    ld = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                gz_sim_launch_file
            ),
            launch_arguments={
                'ns': '',
                'robot_name': launch.substitutions.LaunchConfiguration('model'),
                'global_frame_id': launch.substitutions.LaunchConfiguration('global_frame_id'),
                'odom_frame_id': launch.substitutions.LaunchConfiguration('odom_frame_id')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()