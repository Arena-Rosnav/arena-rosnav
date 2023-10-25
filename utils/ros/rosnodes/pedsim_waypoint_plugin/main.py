#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pedsim_waypoint_plugin.pedsim_waypoint_generator import PedsimWaypointGenerator, WaypointPluginName
import rospy

# Main function.
if __name__ == "__main__":

    print("\n".join(rospy.get_param_names()))

    plugin_name: WaypointPluginName = WaypointPluginName(rospy.get_param("pedsim_waypoint_plugin/plugin_name"))

    rospy.init_node("pedsim_waypoint_generator")
    PedsimWaypointGenerator(plugin_name=plugin_name)
    rospy.spin()
