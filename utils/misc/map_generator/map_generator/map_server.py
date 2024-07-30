#!/usr/bin/env python3
"""
Node that resets the map (amp folder: dynamic_map) to a blank state according to the parameters.
Then starts the nav2_map_server node.
"""
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch.actions import ExecuteProcess
from nav2_common.launch import RewrittenYaml

import os
from map_generator.constants import MAP_FOLDER_NAME, ROSNAV_MAP_FOLDER, MAP_GENERATOR_NS
from map_generator.utils.map import create_empty_map, create_yaml_files
from map_generator.utils.general import load_map_generator_config

class map_server(Node):
    def __init__(self):
        super().__init__('map_server_starter')
        self.declare_parameter('map_file', '')
        self.declare_parameter('single_env', True)
        
        map_file = self.get_parameter('map_file').value
        
        if map_file == MAP_FOLDER_NAME:
            self.prepare_map()
        
        self.start_map_server()

    def prepare_map(self):
        cfg = load_map_generator_config()
        map_properties = self.get_parameter(MAP_GENERATOR_NS('map_properties'), 
                                            cfg['map_properties']).value

        create_empty_map(
            height=map_properties['height'],
            width=map_properties['width'],
            map_name=MAP_FOLDER_NAME,
            dir_path=ROSNAV_MAP_FOLDER,
        )
        create_yaml_files(map_name=MAP_FOLDER_NAME, dir_path=ROSNAV_MAP_FOLDER)

    def start_map_server(self):
        map_yaml_path = os.path.join(ROSNAV_MAP_FOLDER, f'{MAP_FOLDER_NAME}.yaml')
        
        # Create a LaunchDescription
        ld = LaunchDescription()

        # Create nav2_map_server node
        map_server_node = LaunchNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_path}]
        )

        ld.add_action(map_server_node)

        # Execute the launch description
        launch_service = rclpy.launch.LaunchService()
        launch_service.include_launch_description(ld)
        launch_service.run()

def main(args=None):
    rclpy.init(args=args)
    map_server_starter = MapServerStarter()
    rclpy.spin(map_server_starter)
    map_server_starter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()