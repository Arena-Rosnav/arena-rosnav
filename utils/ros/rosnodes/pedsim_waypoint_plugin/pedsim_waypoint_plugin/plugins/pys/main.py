from pedsim_waypoint_plugin.pedsim_waypoint_generator import OutputData, PedsimWaypointGenerator, InputData, WaypointPluginName, WaypointPlugin
import pedsim_msgs.msg
from pysocialforce import Simulator
import numpy as np

@PedsimWaypointGenerator.register(WaypointPluginName.PYS)
class Plugin_PYS(WaypointPlugin):

    def __init__(self):
        self.simulator = None


    def callback(self, data) -> OutputData:
        agent_ids = []
        agent_states = []
        for agent in data.agents:
            agent_ids.append(agent.id)
            agent_states.append((agent.pose.position.x, 
                           agent.pose.position.y, 
                           agent.twist.linear.x, 
                           agent.twist.linear.y, 
                           agent.destination.x, 
                           agent.destination.y, 
                           agent.direction))

        if agent_states == []:
            return [pedsim_msgs.msg.AgentFeedback(unforce=True) for _ in data.agents]
        
        if self.simulator == None:
            self.simulator = Simulator(np.array(agent_states))
        
        feedbacks = []
        
        forces = self.simulator.compute_forces()

        for agent_id, force in zip(agent_ids,forces):
            feedback = pedsim_msgs.msg.AgentFeedback()
            feedback.id = agent_id
            feedback.force.x, feedback.force.y = force[0],force[1]
            feedbacks.append(feedback)
        
        self.simulator = None

        return feedbacks
        
        