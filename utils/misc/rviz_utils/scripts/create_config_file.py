import rospy
import rospkg
import rostopic
import os
import re
import numpy as np
import subprocess
from rviz_utils.config import Config
from rviz_utils.matchers import Matcher
from rviz_utils.utils import Utils
import yaml
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String


class ConfigFileGenerator:
    def __init__(self):
        self.srv_start_setup = rospy.Service("task_generator_setup_finished", Empty, self.start_setup_callback)


    def start_setup_callback(self, _):
        default_file = ConfigFileGenerator.read_default_file()

        displays = [
            Config.MAP,
            Config.TF
        ]

        published_topics, _ = rostopic.get_topic_list()

        published_topics = [topic_info[0] for topic_info in published_topics]

        robot_names = rospy.get_param("robot_names", [])

        for robot_name in robot_names:
            color = Utils.get_random_rviz_color()

            for topic in published_topics:
                config = ConfigFileGenerator.create_display_for_topic(robot_name, topic, color)

                if not config:
                    continue

                displays.append(config)

        if rospy.get_param("pedsim", False):
            displays.append(Config.TRACKED_PERSONS)
            displays.append(Config.TRACKED_GROUPS)
            displays.append(Config.PEDSIM_WALLS)
            displays.append(Config.PEDSIM_WAYPOINTS)

        default_file["Visualization Manager"]["Displays"] = displays

        file_path = ConfigFileGenerator.safe_tmp_config_file(default_file)

        ConfigFileGenerator.send_load_config(file_path)

        return EmptyResponse()

    def create_display_for_topic(robot_name, topic, color):
        matchers = [
            (Matcher.GLOBAL_PLAN, Config.create_path_display),
            (Matcher.LASER_SCAN, Config.create_laser_scan_display),
            (Matcher.GLOBAL_COSTMAP, Config.create_global_map_display),
            (Matcher.LOCAL_COSTMAP, Config.create_local_map_display),
            (Matcher.CURRENT_GOAL, Config.create_pose_display),
            (Matcher.SUBGOAL, Config.create_pose_display),
            (Matcher.MODEL, Config.create_model_display) 
        ]

        for matcher, function in matchers:
            match = re.search(matcher(robot_name), topic)

            if match:
                return function(robot_name, topic, color)

    @staticmethod
    def send_load_config(file_path):
        subprocess.run(f"""rosservice call /rviz/load_config \"path:
            {String(file_path)}\"""", shell=True)

    @staticmethod
    def read_default_file():
        file_path = os.path.join(
            rospkg.RosPack().get_path("rviz_utils"),
            "config",
            "rviz_default.yaml"
        )
        
        with open(file_path) as file:
            return yaml.safe_load(file)

    @staticmethod
    def safe_tmp_config_file(config_file):
        dir_path = os.path.join(
            rospkg.RosPack().get_path("rviz_utils"),
            "tmp"
        )

        try:
            os.mkdir(dir_path)
        except:
            pass

        file_path = os.path.join(
            dir_path,
            "config.rviz"
        )

        with open(file_path, "w") as file:
            yaml.dump(config_file, file)

        return file_path


if __name__ == "__main__":
    rospy.init_node("create_rviz_config_file")

    config_file_generator = ConfigFileGenerator()

    while not rospy.is_shutdown():
        rospy.spin()