from pedsim_waypoint_plugin.pedsim_waypoint_generator import OutputData, PedsimWaypointGenerator, InputData, WaypointPluginName, WaypointPlugin
import pedsim_msgs.msg
from .Integrators import *
from .diff_equation import Diff_Equ
from .Room import Room
import numpy as np


@PedsimWaypointGenerator.register(WaypointPluginName.EVACUATION)
class Plugin_Evacuation(WaypointPlugin):
    def __init__(self):
        ...

    def callback(self, data) -> OutputData:
        '''
        "num_steps" is the duration the simulation will run (recommended:1000)

       "method" is the method of integration. You should use leap_frog even though it will often explode
        since more relaible methods of integration like ode45 and monto carlo take a lot a computational power.
        '''
        N = len(data.agents)                            # quantity of pedestrians aka the number of agents that are currently in the simulation
        
        tau = 1                                         # time-step (s), doesn't seem to affect calculation in our case
        num_steps = 2                                   # the number of force-calculation steps the simulation should go through, "2" equals one step
        room_size = 40                                 # size of square room (m), TODO: integrate real value
        method = leap_frog                              # method used for integration -> leap-frog was the GoTo solution in the original project
        radii = 0.3 * np.ones(N)                        # radii of pedestrians (m) -> was "0.4 * (np.ones(self.N)*variation).squeeze()" before
        m = 180 * np.ones(N)                             # mass of pedestrians (kg) -> was "80 * (np.ones(self.N)*variation).squeeze()" before

        v = np.zeros((2, N, num_steps))                 # Three dimensional array of velocity, not used in leap frog strangely
        y = np.zeros((2, N, num_steps))                 # Three dimensional array of place: x = coordinates(2 dims), y = Agent (N dims), z=Time (2 dims)
        for i in range(N):
            pos_x = data.agents[i].pose.position.x
            pos_y = data.agents[i].pose.position.y
            pos = [pos_x, pos_y] 
            y[:, i, 0] = pos                            # z = 0 -> start position

        """
        obstacles = np.zeros((2, 2, len(data.line_obstacles)))
        for i in range(len(data.line_obstacles)):
            start_pos = [data.line_obstacles[i].start.x, data.line_obstacles[i].start.y]
            end_pos = [data.line_obstacles[i].end.x, data.line_obstacles[i].end.y]
            obstacles[:, :, i] = [start_pos, end_pos]
        """

        room = Room("arena", room_size, data.line_obstacles)                 # kind of room the simulation runs in, added arena room

        """if(data.line_obstacles is not None):
            for i, obstacle in enumerate(data.line_obstacles):
                print(i, obstacle)"""
        
        #create differential equation object
        diff_equ = Diff_Equ(N, room_size, tau, room, radii, m)  # initialize Differential equation

        # calls the method of integration with the starting positions, diffequatial equation, number of steps, and delta t = tau
        y, agents_escaped, forces = method(y[:, :, 0], v[:, :, 0], diff_equ.f, num_steps, tau, room)
        # forces is of shape (2, N, 2)

        feedbacks = []

        for i, agent in enumerate(data.agents):
            #copied from Nhat
            feedback = pedsim_msgs.msg.AgentFeedback()
            feedback.id = agent.id
            feedback.force.x = forces[0][i][0]
            feedback.force.y = forces[1][i][0]
            feedbacks.append(feedback)

        return feedbacks
