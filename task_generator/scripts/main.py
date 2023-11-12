#! /usr/bin/env python3

import rospy
from task_generator.task_generator_node import TaskGenerator

if __name__ == "__main__":
    rospy.init_node("task_generator")

    task_generator = TaskGenerator()

    rospy.spin()
