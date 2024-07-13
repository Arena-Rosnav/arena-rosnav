#! /usr/bin/env python3

from rosros import rospify as rospy
from task_generator.task_generator_node import TaskGenerator

if __name__ == "__main__":
    rospy.init_node("task_generator")

    task_generator = TaskGenerator()

    rospy.spin()
