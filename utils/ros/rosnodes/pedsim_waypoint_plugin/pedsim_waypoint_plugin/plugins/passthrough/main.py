from pedsim_waypoint_plugin.pedsim_waypoint_generator import PedsimWaypointGenerator, AgentStates, WaypointPluginName, WaypointPlugin

@PedsimWaypointGenerator.register(WaypointPluginName.PASSTHROUGH)
class Plugin_Passthrough(WaypointPlugin):
    def __init__(self):
        pass;

    def callback(self, agent_states: AgentStates) -> AgentStates:
        return agent_states