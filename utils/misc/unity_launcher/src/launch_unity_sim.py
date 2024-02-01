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
    # arena simulation setup path
    args += [
        "-arena_sim_setup_path",
        os.path.join(ws_src_path, "arena-simulation-setup")
    ]
    # headless arg
    headless = rospy.get_param('~headless', False)
    if isinstance(headless, bool) and headless:
        args += ["-batchmode"]
    # namespace
    namespace = rospy.get_param("~namespace", "")
    args += [
        "-namespace",
        namespace
    ]
    # tcp ip & port
    tcp_ip = str(rospy.get_param("~tcp_ip", "128.0.0.1"))
    tcp_port = str(rospy.get_param("~tcp_port", "11000"))
    args += [
        "-tcp_ip",
        tcp_ip,
        "-tcp_port",
        tcp_port
    ]

    # test
    args += ["-force-vulkan"]
    args += ["-logFile", f"/home/maxmilian/loggging/{namespace}_log.txt"]

    subprocess.run([unity_executable_path] + args, check=True)


if __name__ == '__main__':
    try:
        launch_unity()
    except rospy.ROSInterruptException:
        pass
