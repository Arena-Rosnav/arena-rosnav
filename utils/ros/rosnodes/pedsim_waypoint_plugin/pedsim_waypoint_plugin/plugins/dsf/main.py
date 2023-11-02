from pedsim_waypoint_plugin.pedsim_waypoint_generator import OutputData, PedsimWaypointGenerator, InputData, WaypointPluginName, WaypointPlugin
import pedsim_msgs.msg
from socialforce import Simulator
from socialforce.potentials import *

@PedsimWaypointGenerator.register(WaypointPluginName.DSF)
class Plugin_DSF(WaypointPlugin):

    steps = 1
    buffer_length = 7

    def __init__(self):
        self.simulator = Simulator(ped_ped=PedPedPotential(sigma=1))
        self.old_forces = None
        self.force_buffers = None
        self.start = True

    def callback(self, data) -> OutputData:
        agent_ids = []
        agent_states = []
        for agent in data.agents:
            agent_ids.append(agent.id)
            agent_states.append((agent.pose.position.x, 
                           agent.pose.position.y, 
                           agent.twist.linear.x, 
                           agent.twist.linear.y, 
                           agent.acceleration.x, 
                           agent.acceleration.y, 
                           agent.destination.x, 
                           agent.destination.y, 
                           agent.direction,
                           7)) #desired speed???
        
        if agent_states == []:
            self.start = True
            return [pedsim_msgs.msg.AgentFeedback(unforce=True) for _ in data.agents]
        
        state = self.simulator.normalize_state(agent_states)
        
        if self.start:
            self.force_buffers = {agent_id:ForceBuffer(self.buffer_length) for agent_id in agent_ids}
            self.old_forces = {agent_id:Force() for agent_id in agent_ids}
            states = self.simulator.run(state,self.buffer_length)
            for state in states:
                for i,agent_id in enumerate(agent_ids):
                    self.force_buffers[agent_id].apply(Force(state[i][4],state[i][5]))
                    self.force_buffers[agent_id].pop()
            self.start = False
            return [pedsim_msgs.msg.AgentFeedback(unforce=True) for _ in data.agents]
        
        states = self.simulator.run(state,self.steps)
        next_state = states[-1]

        feedbacks = []

        for agent_id, state in zip(agent_ids,next_state):
            feedback = pedsim_msgs.msg.AgentFeedback()
            feedback.id = agent_id

            force_new = Force(state[4],state[5])
            #forces_diff = force_new - self.old_forces[agent_id]
            self.old_forces[agent_id] = force_new
            force_pub = self.force_buffers[agent_id].pop()
            self.force_buffers[agent_id].apply(force_new)

            feedback.force.x, feedback.force.y = force_pub.x, force_pub.y
            feedbacks.append(feedback)

        return feedbacks
    
class Force:
    
    def __init__(self, x=0,y=0):
        self.x = x
        self.y = y
    
    def __add__(self,force):
        return Force(self.x+force.x, self.y+force.y)
    
    def __sub__(self,force):
        return self + force*(-1)
    
    def __mul__(self, scale):
        return Force(self.x*scale, self.y*scale)
    
    def __iadd__(self,force):
        self.x += force.x
        self.y += force.y
        return self
    
    def __truediv__(self, scale):
        return self*(1/scale)
    
    def __eq__(self, force):
        return self.x == force.x and self.y == force.y
    
class ForceBuffer:

    def __init__(self, size: int):
        self.__buffer = [Force() for _ in range(size)]

    def pop(self) -> Force:
        self.__buffer.append(Force())
        return self.__buffer.pop(0)

    def apply(self, force: Force):
        for buffer_force in self.__buffer:
            buffer_force += force / len(self.__buffer)
        # extension: scale force non-constantly based on position with for i, buffer_force in enumerate(self.__buffer)
    
    def add(self, force: Force):
        for buffer_force in self.__buffer:
            buffer_force = (buffer_force+force)/2
  


