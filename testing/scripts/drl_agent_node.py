#!/usr/bin/env python3
import argparse
import os
import sys
from urllib import parse

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from rl_utils.utils.observation_collector.observation_manager import ObservationManager
from rosgraph_msgs.msg import Clock
from rosnav import *
from rosnav.msg import ResetStackedObs
from rosnav.srv import GetAction, GetActionRequest
from task_generator.shared import Namespace

sys.modules["rl_agent"] = sys.modules["rosnav"]


ACTION_FREQUENCY = 4  # in Hz


class RosnavActionNode:
    def __init__(self, ns: Namespace = ""):
        """Initialization procedure for the DRL agent node.

        Args:
            ns (str, optional):
                Simulation specific ROS namespace. Defaults to None.
        """
        self.ns = Namespace(ns)

        self._action_pub = rospy.Publisher(f"{self.ns}/cmd_vel", Twist, queue_size=1)
        self._action_period_time = RosnavActionNode._get_action_frequency_in_nsecs()
        self.last_time = 0

        rospy.wait_for_service(f"{self.ns}/rosnav/get_action")
        self._get_action_srv = rospy.ServiceProxy(
            f"{self.ns}/rosnav/get_action", GetAction
        )

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

    def _get_and_publish_next_action(self) -> None:
        msg = GetActionRequest()

        action = self._get_action_srv(msg).action
        self._publish_action(action)

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


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("-ns", "--namespace", type=str)

    return parser.parse_known_args()[0]


if __name__ == "__main__":
    rospy.init_node(f"RosnavActionNode", anonymous=True)

    args = parse_args()

    agent = RosnavActionNode(ns=args.namespace)

    while not rospy.is_shutdown():
        rospy.spin()
