#! /usr/bin/env python3

import os
from typing import Optional
from rosros import rospify as rospy
import rospkg
import yaml

from task_generator.cfg import TaskGeneratorConfig

def load_parameters_from_yaml(file_path: str):
    with open(file_path, 'r') as file:
        params = yaml.safe_load(file)
    
    if params and 'ros__parameters' in params:
        for key, value in params['ros__parameters'].items():
            rospy.set_param(key, value)

def run(namespace: Optional[str] = None):
    if namespace is None:
        namespace = str(rospy.get_name())

    rospack = rospkg.RosPack()
    config_file_path = os.path.join(rospack.get_path('arena_bringup'), 'configs', 'task_generator.yaml')

    load_parameters_from_yaml(config_file_path)

    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("task_generator_server")
    run()
