#!/usr/bin/env python
"""This module handles the launch of the unity simulator"""

import subprocess
import os
import rospy


def launch_unity():
    """Launches the unity executable with given arguments
    """
    rospy.init_node('unity_sim_node', anonymous=True)

    dev = rospy.get_param('~development_mode', False)
    if isinstance(dev, bool) and dev:
        # Unity should be launched trough Unity Editor in dev mode
        return

    # find unity executable
    current_path = os.path.dirname(os.path.abspath(__file__))
    ws_src_path = os.path.join(current_path, "../../../../..")
    unity_executable_path = os.path.join(ws_src_path, "arena-unity/Build/arena-unity")

    # args
    args = list()
    args += [
        "-arena_sim_setup_path",
        os.path.join(ws_src_path, "arena-simulation-setup")
    ]
    headless = rospy.get_param('~headless', False)
    if isinstance(headless, bool) and headless:
        args += ["-batchmode"]

    # test
    args += ["-force-vulkan"]


    subprocess.run([unity_executable_path] + args, check=True)


if __name__ == '__main__':
    try:
        launch_unity()
    except rospy.ROSInterruptException:
        pass
