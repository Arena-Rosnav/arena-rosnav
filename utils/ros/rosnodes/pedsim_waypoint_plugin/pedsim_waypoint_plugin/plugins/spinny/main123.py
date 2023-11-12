import numpy as np
from pedsim_waypoint_plugin.pedsim_waypoint_generator import OutputData, PedsimWaypointGenerator, InputData, WaypointPluginName, WaypointPlugin
import pedsim_msgs.msg

@PedsimWaypointGenerator.register(WaypointPluginName.SPINNY)
class Plugin_Spinny(WaypointPlugin):

    offset = 45
    sign = 1

    def __init__(self):
        ...

    def callback(self, data) -> OutputData:
        
        def datapoint_to_vec(agent: pedsim_msgs.msg.AgentState) -> pedsim_msgs.msg.AgentFeedback:
            
            angle = agent.direction

            if abs(angle - 2 * np.pi) < 1e-2:
                self.sign *= -1

            velocity = np.linalg.norm(np.array([agent.twist.linear.x, agent.twist.linear.y]))

            feedback = pedsim_msgs.msg.AgentFeedback()

            feedback.id = agent.id
            feedback.force.x = np.cos(angle + self.sign * self.offset * np.pi/180) * velocity
            feedback.force.y = np.sin(angle + self.sign * self.offset * np.pi/180) * velocity

            #print("id:",agent.id,"fx:",feedback.force.x, "fy:",feedback.force.y, "angle:",angle)

            return feedback

        return [datapoint_to_vec(agent) for agent in data.agents]
