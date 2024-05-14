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


ACTION_FREQUENCY = 5  # in Hz


class RosnavActionNode:
    def __init__(self, ns: Namespace = ""):
        """Initialization procedure for the DRL agent node.

        Args:
            ns (str, optional):
                Simulation specific ROS namespace. Defaults to None.
        """
        self.ns = Namespace(ns) if ns else Namespace(rospy.get_namespace()[:-1])
        rospy.loginfo(f"Starting Rosnav-Action-Node on {self.ns}")

        self._action_pub = rospy.Publisher(f"{self.ns}/cmd_vel", Twist, queue_size=1)
        rospy.wait_for_service(f"{self.ns}/rosnav/get_action")
        self._get_action_srv = rospy.ServiceProxy(
            f"{self.ns}/rosnav/get_action", GetAction
        )

        frequency = rospy.get_param("action_frequency", ACTION_FREQUENCY)
        self._rate = rospy.Rate(frequency)

        while not rospy.is_shutdown():
            self._get_and_publish_next_action()
            self._rate.sleep()

    def _get_and_publish_next_action(self) -> None:
        action = self._get_action_srv(GetActionRequest()).action
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

    parser.add_argument("-ns", "--namespace", type=str, default="")

    return parser.parse_known_args()[0]


if __name__ == "__main__":
    rospy.init_node(f"RosnavActionNode", anonymous=True)

    args = parse_args()

    agent = RosnavActionNode(ns=args.namespace)

    while not rospy.is_shutdown():
        rospy.spin()
