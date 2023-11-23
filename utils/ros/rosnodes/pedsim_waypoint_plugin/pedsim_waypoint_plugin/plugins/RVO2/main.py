from pedsim_waypoint_plugin.pedsim_waypoint_generator import OutputData, PedsimWaypointGenerator, InputData, WaypointPluginName, WaypointPlugin
import pedsim_msgs.msg
import rvo2

@PedsimWaypointGenerator.register(WaypointPluginName.RVO2)
class Plugin_RVO2(WaypointPlugin):
    def __init__(self):
        self.sim = rvo2.PyRVOSimulator(1/60.,1.5,1.5,2,0.4,2)

    def callback(self, data) -> OutputData:
    	self.sim.doStep()
        return [pedsim_msgs.msg.AgentFeedback(unforce=False) for agent in data.agents]

