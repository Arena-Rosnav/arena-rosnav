from pedsim_waypoint_plugin.pedsim_waypoint_generator import OutputData, PedsimWaypointGenerator, InputData, WaypointPluginName, WaypointPlugin

@PedsimWaypointGenerator.register(WaypointPluginName.PASSTHROUGH)
class Plugin_Passthrough(WaypointPlugin):
    def __init__(self):
        pass;

    def callback(self, data) -> OutputData:
        return data.agents