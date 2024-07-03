#! /usr/bin/env python3
from urllib import parse
import numpy as np
import rospy
import argparse
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan

"""
This node inspects the scanner messages for any obstacles in robots proximity zones
and slows down, makes a full stop accordingly.
"""


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("robot_namespace")

    return parser.parse_known_args()[0]

class EmergencyBrakeNode:
    def __init__(self, namespace):
        """
        __init__ [The node expects the commanded velocity to be published on /nav_vel, then redirects to /cmd_vel]

        """
        self.scan_sub = node.create_subscription(LaserScan, "/scan", self.scan_callback)
        self.nav_vel_sub = node.create_subscription(Twist, "/nav_vel", self.nav_vel_callback)
        self.cmd_vel_pub = node.create_publisher(Twist, queue_size=1, latch=True, "/cmd_vel")

    def scan_callback(self, data):
        """
        scan_callback [saves scan message for later use]

        Args:
            data ([LaserScan]): [scan data]
        """
        self.curr_laser_data = data

    def nav_vel_callback(self, data):
        """
        nav_vel_callback [Checks for obstacles near the robot, adapts the commanded velocity]

        Args:
            data ([Twist]): [The desired velocity command]
        """

        scan_data = self.curr_laser_data
        range_min = scan_data.range_min
        ranges = np.asarray(scan_data.ranges)

        ## Checking for obstacles in the restricted zone if any then stop moving
        proximity_ranges = (ranges <= 0.5) & (ranges >= range_min)
        res = np.any(proximity_ranges)
        if res:
            self.cmd_vel_pub.publish(Twist())
            return

        ## Checking for any obstacles in the hazard zone and slowing down accordingly
        proximity_ranges = (ranges <= 1.0) & (ranges >= range_min)
        res = np.any(proximity_ranges)
        if res:
            data.linear.x *= 0.5
            self.cmd_vel_pub.publish(data)
            return
        self.cmd_vel_pub.publish(data)


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("emergency_break_node", anonymous=True)

    args = parse_args()

    node = EmergencyBrakeNode(args.robot_namespace)
    
    while not rospy.is_shutdown():
        rospy.spin()
