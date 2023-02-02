#!/usr/bin/env python3

# general packages
from math import nan
from threading import current_thread
import time
import numpy as np
import csv
import os
import sys
import math
import re
from rosgraph_msgs.msg import Clock
from rospy.core import traceback
import rostopic
import rospkg
import subprocess
from datetime import datetime
import rosparam
import yaml

# ros packages
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose2D, Pose, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# for transformations
from tf.transformations import euler_from_quaternion


class DataCollector:
    def __init__(self, topic):
        topic_callbacks = [
            ("scan", self.laserscan_callback),
            ("odom", self.odometry_callback),
            ("cmd_vel", self.action_callback),
        ]

        try:
            callback = lambda msg: [t[1] for t in topic_callbacks if t[0] == topic[1]][0](msg)
        except:
            traceback.print_exc()
            return

        self.full_topic_name = topic[1]
        self.data = None

        print(topic[0])

        self.subscriber = rospy.Subscriber(topic[0], topic[2], callback)

    def episode_callback(self, msg_scenario_reset):
        print(msg_scenario_reset)
        
        self.data = msg_scenario_reset.data

    def laserscan_callback(self, msg_laserscan: LaserScan):
        self.data = [msg_laserscan.range_max if math.isnan(val) else round(val, 3) for val in msg_laserscan.ranges]

    def odometry_callback(self, msg_odometry: Odometry):
        pose3d = msg_odometry.pose.pose
        twist = msg_odometry.twist.twist

        self.data = {
            "position": [
                round(val, 3) for val in [
                    pose3d.position.x,
                    pose3d.position.y,
                    euler_from_quaternion(
                        [
                            pose3d.orientation.x, 
                            pose3d.orientation.y,
                            pose3d.orientation.z,
                            pose3d.orientation.w
                        ]
                    )[2]
                ]
            ],
            "velocity": [
                round(val, 3) for val in [
                    twist.linear.x,
                    twist.linear.y,
                    twist.angular.z
                ]
            ]
        }

    def action_callback(self, msg_action: Twist): # variables will be written to csv whenever an action is published
        self.data = [
            round(val, 3) for val in [
                msg_action.linear.x,
                msg_action.linear.y,
                msg_action.angular.z
            ]
        ]

    def get_data(self):
        return (
            self.full_topic_name,
            self.data 
        )


class Recorder:
    def __init__(self):
        self.model = rospy.get_param(os.path.join(rospy.get_namespace(), "model"), "")

        self.dir = rospkg.RosPack().get_path("evaluation")
        self.result_dir = os.path.join(self.dir, "data", datetime.now().strftime("%d-%m-%Y_%H-%M-%S")) + "_" + rospy.get_namespace().replace("/", "")

        try:
            os.mkdir(self.result_dir)
        except:
            pass
        
        self.write_params()

        topics_to_monitor = self.get_topics_to_monitor()

        topics = rostopic.get_topic_list()
        published_topics = topics[0]

        topic_matcher = re.compile(f"{rospy.get_namespace()}({'|'.join([t[0] for t in topics_to_monitor])})$")

        topics_to_sub = []

        for t in published_topics:
            topic_name = t[0]

            match = re.search(topic_matcher, topic_name)

            if not match: 
                continue

            print(match, t, topic_matcher, match.group())

            topics_to_sub.append([topic_name, *self.get_class_for_topic_name(topic_name)])

            # topics_to_sub.append([topic_name, *[t for t in topics_to_monitor if t[0] == match.group()][0]])

        self.data_collectors = []

        for topic in topics_to_sub:
            self.data_collectors.append(DataCollector(topic))
            self.write_data(
                topic[1], [
                    "time", "data"
                ],
                mode="w"
            )

        self.write_data("episode", ["time", "episode"], mode="w")
        self.write_data("start_goal", ["episode", "start", "goal"], mode="w")

        self.current_episode = 0

        self.config = self.read_config()

        self.clock_sub = rospy.Subscriber("/clock", Clock, self.clock_callback)
        self.scenario_reset_sub = rospy.Subscriber("/scenario_reset", Int16, self.scenario_reset_callback)

        self.current_time = None

        print(rosparam.print_params("", "/"))

    def scenario_reset_callback(self, data: Int16):
        self.current_episode = data.data

    def clock_callback(self, clock: Clock):
        current_simulation_action_time = clock.clock.secs * 10e9 + clock.clock.nsecs

        if not self.current_time:
            self.current_time = current_simulation_action_time

        time_diff = (current_simulation_action_time - self.current_time) / 1e6 ## in ms

        if time_diff < self.config["record_frequency"]:
            return

        self.current_time = current_simulation_action_time

        for collector in self.data_collectors:
            topic_name, data = collector.get_data()
            
            self.write_data(topic_name, [self.current_time, data])
        
        self.write_data("episode", [self.current_time, self.current_episode])
        self.write_data("start_goal", [
            self.current_episode, 
            rospy.get_param(rospy.get_namespace() + "start", [0, 0, 0]), 
            rospy.get_param(rospy.get_namespace() + "goal", [0, 0, 0])
        ])

    def read_config(self):
        with open(self.dir + "/data_recorder_config.yaml") as file:
            return yaml.safe_load(file)

    def get_class_for_topic_name(self, topic_name):
        if "/scan" in topic_name:
            return ["scan", LaserScan]
        if "/odom" in topic_name:
            return ["odom", Odometry]
        if "/cmd_vel" in topic_name:
            return ["cmd_vel", Twist]

    def get_topics_to_monitor(self):
        return [
            ("scan", LaserScan),
            ("scenario_reset", Int16),
            ("odom", Odometry),
            ("cmd_vel", Twist)
        ]

    def write_data(self, file_name, data, mode="a"):
        with open(f"{self.result_dir}/{file_name}.csv", mode, newline = "") as file:
            writer = csv.writer(file, delimiter = ',')
            writer.writerow(data)
            file.close()
    
    def write_params(self):
        with open(self.result_dir + "/params.yaml", "w") as file:
            yaml.dump({
                "model": self.model,
                "map_file": rospy.get_param("/map_file", ""),
                "scenario_file": rospy.get_param("/scenario_file", ""),
                "local_planner": rospy.get_param(rospy.get_namespace() + "local_planner"),
                "agent_name": rospy.get_param(rospy.get_namespace() + "agent_name", ""),
                "namespace": rospy.get_namespace().replace("/", "")
            }, file)


if __name__=="__main__":
    rospy.init_node("data_recorder", anonymous=True) 

    time.sleep(5)   

    Recorder()

    rospy.spin()

