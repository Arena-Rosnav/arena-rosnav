#!/usr/bin/env python
"""This module handles the launch of the unity simulator"""

import subprocess
import os
import rospy


def launch_unity():
    """Launches the unity executable with given arguments
    """
    rospy.init_node('unity_sim_node', anonymous=True)

    unity_executable_path = "/path/to/your/project.x86_64"

    # find unity executable
    current_path = os.path.dirname(os.path.abspath(__file__))
    ws_src_path = os.path.join(current_path, "../../../../..")
    unity_executable_path = os.path.join(ws_src_path, "arena-unity/Build/arena-unity")

    subprocess.run([unity_executable_path], check=True)


if __name__ == '__main__':
    try:
        launch_unity()
    except rospy.ROSInterruptException:
        pass
