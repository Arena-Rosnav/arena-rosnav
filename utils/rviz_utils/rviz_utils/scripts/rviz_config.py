#! /usr/bin/env python3

import os
import re
import subprocess
import sys
import tempfile
import time

import launch
import launch.launch_service
import launch_ros.actions
import rcl_interfaces.srv
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rviz_utils.config import Config
from rviz_utils.matchers import Matcher
from rviz_utils.utils import Utils
from std_msgs.msg import String
from std_srvs.srv import Empty


class ConfigFileGenerator(Node):
    def __init__(self):
        Node.__init__(self, 'create_rviz_config_file')

        TASKGEN_NODE = '/task_generator_node'
        TASKGEN_PARAM_SRV = os.path.join(TASKGEN_NODE, 'get_parameters')
        PARAM_INITIALIZED = 'initialized'

        get_parameters_cli = self.create_client(rcl_interfaces.srv.GetParameters, TASKGEN_PARAM_SRV)
        while not get_parameters_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'waiting for service {TASKGEN_PARAM_SRV} to become available')
        self.get_logger().info(f'service {TASKGEN_PARAM_SRV} is available')

        while True:
            req = rcl_interfaces.srv.GetParameters.Request(names=[PARAM_INITIALIZED])
            future = get_parameters_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            params = future.result()
            if params.values and params.values[0].bool_value:
                break
            self.get_logger().info(f'waiting for {PARAM_INITIALIZED} to be set')
            time.sleep(1)
        self.get_logger().info(f'param {PARAM_INITIALIZED} is set')

        # self.cli_load = self.create_client('/rviz2/load_config', rcl_interfaces.srv.SetString)

    def create_config(self) -> str:
        default_file = ConfigFileGenerator._read_default_file()

        displays = [
            #     Config.MAP,
            #     Config.TF
        ]

        published_topics = [topic[0] for topic in self.get_topic_names_and_types()]

        
        # Create Robot-Groups based on robot names
        self.declare_parameter('robot_names', [])
        robot_names = self.get_parameter('robot_names').value
        
        for robot_name in robot_names:
            robot_group = self._create_robot_group(robot_name)
            displays.append(robot_group)

        
        robot_names = self.get_parameter_or("robot_names.value", [])

        for robot_name in robot_names:
            color = Utils.get_random_rviz_color()

            for topic in published_topics:
                config = self._create_display_for_topic(robot_name, topic, color)

                if not config:
                    continue

                displays.append(config)

        
        if self.get_parameter_or("pedsim.value", False):
            displays.append(Config.TRACKED_PERSONS)
            displays.append(Config.TRACKED_GROUPS)
            displays.append(Config.PEDSIM_WALLS)
            displays.append(Config.PEDSIM_WAYPOINTS)

        default_file["Visualization Manager"]["Displays"] = displays

        file_path = self._tmp_config_file(default_file)
        self.get_logger().info(f'created config file at {file_path}')

        return file_path

    def _start_setup_callback(self, request, response):
        self.get_logger().info("Service callback triggered.")
        file_path = self.create_config()
        self._send_load_config(file_path)
        return response

    def _create_robot_group(self, robot_name):
        """Creates a Robot Group for the Visualisation"""
        color = Utils.get_random_rviz_color()
        
        robot_group = {
            'Class': 'rviz_common/Group',
            'Name': f'Robot: {robot_name}',
            'Enabled': True,
            'Displays': []
        }
        
        # Odometry - Visualisation
        odom_topic = f'/task_generator_node/{robot_name}/odom'
        odom_display = {
            'Class': 'rviz_default_plugins/Odometry',
            'Name': 'Odometry',
            'Enabled': True,
            'Topic': odom_topic,
            'Shape': 'Arrow',
            'Color': color,
            'Keep': 100
        }
        
        robot_group['Displays'].append(odom_display)
        return robot_group

    def _create_display_for_topic(self, robot_name, topic, color):
        matchers = [
            (Matcher.GLOBAL_PLAN, Config.create_path_display),
            (Matcher.LASER_SCAN, Config.create_laser_scan_display),
            (Matcher.GLOBAL_COSTMAP, Config.create_global_map_display),
            (Matcher.LOCAL_COSTMAP, Config.create_local_map_display),
            (Matcher.GOAL, Config.create_pose_display),
            (Matcher.MODEL, Config.create_model_display)
        ]

        for matcher, function in matchers:
            match = re.search(matcher(robot_name), topic)

            if match:
                return function(robot_name, topic, color)

    def _send_load_config(self, file_path):
        # print(f"Attempting to call /rviz/load_config with file: {file_path}")
        while not self.cli_load.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for service /rviz/load_config to become available')
        self.cli_load.call(file_path)
        # print("Call to /rviz/load_config completed.")

    def get_parameter_or(self, param_name, default_value):
       
        try:
            return self.get_parameter(param_name).value
        except:
            return default_value

    @staticmethod
    def _read_default_file():
        package_path = get_package_share_directory("rviz_utils")
        file_path = os.path.join(package_path, "config", "rviz_default.rviz")

        with open(file_path) as file:
            return yaml.safe_load(file)

    @classmethod
    def _tmp_config_file(cls, config_file):
        f = tempfile.NamedTemporaryFile('w', delete=False)
        yaml.dump(config_file, f)
        f.close()
        return f.name


def main():
    rclpy.init()

    config_file_generator = ConfigFileGenerator()
    try:
        config_file = config_file_generator.create_config()
        launch_service = launch.launch_service.LaunchService()
        launch_service.include_launch_description(
            launch.LaunchDescription([
                launch_ros.actions.Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    arguments=['-d', config_file, '--ros-args', '--clock'],
                    parameters=[{"use_sim_time": True}],
                    output="screen",
                )
            ])
        )
        launch_service.run()
    except KeyboardInterrupt:
        pass
    finally:
        config_file_generator.destroy_node()
        rclpy.shutdown()

    sys.exit(0)


if __name__ == "__main__":
    main()