from pedsim_waypoint_plugin.pedsim_waypoint_generator import OutputData, PedsimWaypointGenerator, InputData, WaypointPluginName, WaypointPlugin
import pedsim_msgs.msg

@PedsimWaypointGenerator.register(WaypointPluginName.PASSTHROUGH)
class Plugin_Passthrough(WaypointPlugin):

    def __init__(self):
        ...

    def callback(self, data) -> OutputData:
        return [pedsim_msgs.msg.AgentFeedback(unforce=True) for agent in data.agents]