#!/usr/bin/env python3
import argparse
import os
import sys
from urllib import parse

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from rl_utils.utils.observation_collector import ObservationCollector
from rosgraph_msgs.msg import Clock
from rosnav import *
from rosnav.msg import ResetStackedObs
from rosnav.srv import GetAction, GetActionRequest
from std_msgs.msg import Int16

sys.modules["rl_agent"] = sys.modules["rosnav"]


ACTION_FREQUENCY = 4  # in Hz


class DeploymentDRLAgent:
    def __init__(self, ns=None):
        """Initialization procedure for the DRL agent node.

        Args:
            ns (str, optional):
                Simulation specific ROS namespace. Defaults to None.
        """
        self.observation_collector = ObservationCollector(
            ns, rospy.get_param("laser/num_beams"), external_time_sync=False
        )

        self._max_laser_range = rospy.get_param(os.path.join(ns, "laser", "range"))

        self._action_pub = rospy.Publisher(f"{ns}/cmd_vel", Twist, queue_size=1)
        self._reset_stacked_obs_pub = rospy.Publisher(
            f"{ns}/rosnav/reset_stacked_obs", ResetStackedObs, queue_size=1
        )

        self.ns = ns

        self._action_period_time = DeploymentDRLAgent._get_action_frequency_in_nsecs()
        self.last_action = [0, 0, 0]
        self.last_time = 0

        rospy.wait_for_service(f"{ns}/rosnav/get_action")
        self._get_action_srv = rospy.ServiceProxy(f"{ns}/rosnav/get_action", GetAction)

        rospy.Subscriber("/scenario_reset", Int16, self._episode_reset_callback)
        rospy.Subscriber("/clock", Clock, callback=self._clock_callback)

    @staticmethod
    def _get_action_frequency_in_nsecs():
        action_period_time = 1 / rospy.get_param("action_frequency", ACTION_FREQUENCY)

        return action_period_time * 100000000

    def _clock_callback(self, data):
        curr_time = data.clock.secs * (10**12) + data.clock.nsecs

        if curr_time - self.last_time <= self._action_period_time:
            return

        self._get_and_publish_next_action()

    def _episode_reset_callback(self, data: Int16):
        obs = self.observation_collector.get_observations(ns_prefix=self.ns)
        msg = self.build_get_action_request(obs)
        self._reset_stacked_obs_pub.publish(msg)

    def _get_and_publish_next_action(self) -> None:
        obs = self.observation_collector.get_observations(ns_prefix=self.ns)
        msg = self.build_get_action_request(obs)

        try:
            action = self._get_action_srv(msg).action
            self.last_action = action
            self._publish_action(action)
        except:
            # print("Error")
            # print(msg)
            pass

    def _publish_action(self, action):
        """Publishes an action on 'self._action_pub' (ROS topic).
        Args:
            action (np.ndarray):
                Action in [linear velocity, angular velocity]
        """
        action_msg = Twist()

        action_msg.linear.x = action[0]
        action_msg.linear.y = action[1]
        action_msg.angular.z = action[2]

        self._action_pub.publish(action_msg)

    def build_get_action_request(self, obs: dict):
        msg = GetActionRequest()
        msg.laser_scan = np.clip(obs["laser_scan"], 0, self._max_laser_range)
        msg.goal_in_robot_frame = obs["goal_in_robot_frame"]
        msg.last_action = self.last_action

        return msg


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("-ns", "--namespace", type=str)

    return parser.parse_known_args()[0]


if __name__ == "__main__":
    rospy.init_node(f"DRL_local_planner", anonymous=True)

    args = parse_args()

    agent = DeploymentDRLAgent(ns=args.namespace)

    while not rospy.is_shutdown():
        rospy.spin()
