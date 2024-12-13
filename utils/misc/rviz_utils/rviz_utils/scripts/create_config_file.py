#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import os
import re
import subprocess
from ament_index_python.packages import get_package_share_directory
from rviz_utils.config import Config
from rviz_utils.matchers import Matcher
from rviz_utils.utils import Utils
import yaml
from std_srvs.srv import Empty
from std_msgs.msg import String


class ConfigFileGenerator(Node):
    def __init__(self):
            super().__init__('create_rviz_config_file')
            self.srv_start_setup = self.create_service(
            Empty, "task_generator_setup_finished", self.start_setup_callback)

    def start_setup_callback(self, request, response):
        self.get_logger().info("Service callback triggered.")
        default_file = ConfigFileGenerator.read_default_file()

        displays = [
        #     Config.MAP,
        #     Config.TF
        ]

        published_topics = [topic[0] for topic in self.get_topic_names_and_types()]

        robot_names = self.get_parameter_or("robot_names.value", [])

        for robot_name in robot_names:
            color = Utils.get_random_rviz_color()

            for topic in published_topics:
                config = ConfigFileGenerator.create_display_for_topic(
                    robot_name, topic, color)

                if not config:
                    continue

                displays.append(config)

        if self.get_parameter_or("pedsim.value", False):
            displays.append(Config.TRACKED_PERSONS)
            displays.append(Config.TRACKED_GROUPS)
            displays.append(Config.PEDSIM_WALLS)
            displays.append(Config.PEDSIM_WAYPOINTS)

        default_file["Visualization Manager"]["Displays"] = displays

        file_path = ConfigFileGenerator.safe_tmp_config_file(default_file)

        print(f"Attempting to call /rviz/load_config with file: {file_path}")
        subprocess.run(f"""ros2 service call /rviz/load_config_file rcl_interfaces/srv/SetString "data: '{file_path}'" """, shell=True)
        print("Call to /rviz/load_config completed.")
        return response
    def create_display_for_topic(robot_name, topic, color):
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

    @staticmethod
    def send_load_config(file_path):
        subprocess.run(f"""ros2 service call /rviz/load_config std_srvs/srv/Empty {{}}""", shell=True)

    @staticmethod
    def read_default_file():
        package_path = get_package_share_directory("rviz_utils")
        file_path = os.path.join(package_path, "config", "rviz_default.rviz")
    
        with open(file_path) as file:
            return yaml.safe_load(file)

    @staticmethod
    def safe_tmp_config_file(config_file):
        dir_path = os.path.join(
            get_package_share_directory("rviz_utils"),
            "tmp"
        )

        try:
            os.mkdir(dir_path)
        except BaseException:
            pass

        file_path = os.path.join(
            dir_path,
            "config.rviz"
        )

        with open(file_path, "w") as file:
            yaml.dump(config_file, file)

        return file_path

def main(): 
    rclpy.init()
    config_file_generator = ConfigFileGenerator()
    
    try:
        rclpy.spin(config_file_generator)
    except KeyboardInterrupt:
        pass
    finally:
        config_file_generator.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
