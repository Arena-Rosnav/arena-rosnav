from pedsim_waypoint_plugin.pedsim_waypoint_generator import OutputData, PedsimWaypointGenerator, InputData, WaypointPluginName, WaypointPlugin
import pedsim_msgs.msg
import Integrators
from diff_equation import Diff_Equ
from Room import Room
import numpy as np


@PedsimWaypointGenerator.register(WaypointPluginName.EVACUATION)
class Plugin_Evacuation(WaypointPlugin):
    def __init__(self):
        ...

    def callback(self, data) -> OutputData:
        def calculate_forces(agent: pedsim_msgs.msg.AgentState) -> pedsim_msgs.msg.AgentFeedback:
            '''
            "num_steps" is the duration the simulation will run (recommended:1000)

            "method" is the method of integration. You should use leap_frog even though it will often explode
            since more relaible methods of integration like ode45 and monto carlo take a lot a computational power.
            '''
            tau=0.1                                         # time-step (s), TODO: figure out right value
            num_steps = 1                                   # the number of force-calculation steps the simulation should go through (each callback should be 1 step?)
            method = getattr(Integrators, "leap_frog")      # method used for integration -> leap-frog was the GoTo solution in the original project
            N = len(data.agents)  # quantity of pedestrians aka the number of agents that are currently in the simulation
            v = np.zeros((2, N, num_steps))                 # Three dimensional array of velocity, TODO: figure out right value
            y = np.zeros((2, N, num_steps))                 # Three dimensional array of place: x = coordinates, y = Agent, z=Time, TODO: figure out right value
            room_size = 20                                  # size of square room (m), TODO: has to be deleted or changed
            room = Room("square", room_size)                # kind of room the simulation runs in, TODO: has to be deleted or changed
            radii = np.ones(N)                              # radii of pedestrians (m), TODO: figure out right value -> was "0.4 * (np.ones(self.N)*variation).squeeze()" before
            m = np.ones(N)                                  # mass of pedestrians (kg), TODO: figure out right value -> was "80 * (np.ones(self.N)*variation).squeeze()" before

            diff_equ = Diff_Equ(N, room_size, tau, room, radii, m)  # initialize Differential equation

            # calls the method of integration with the starting positions, diffequatial equation, number of steps, and delta t = tau
            y, agents_escaped, forces = method(y[:, :, 0], v[:, :, 0], diff_equ.f, num_steps, tau, room)
            
            #TODO: Calculate x/y-Forces correctly
            feedback = pedsim_msgs.msg.AgentFeedback()
            feedback.id = agent.id
            feedback.force.x = forces[0]
            feedback.force.y = forces[1]

            return feedback
        
        return [calculate_forces(agent) for agent in data.agents]
