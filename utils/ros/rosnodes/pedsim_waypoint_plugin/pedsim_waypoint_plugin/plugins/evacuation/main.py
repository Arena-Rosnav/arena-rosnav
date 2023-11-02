from pedsim_waypoint_plugin.pedsim_waypoint_generator import OutputData, PedsimWaypointGenerator, InputData, WaypointPluginName, WaypointPlugin
import pedsim_msgs.msg
from Integrators import *
from Simulation_class import Simulation


@PedsimWaypointGenerator.register(WaypointPluginName.EVACUATION)
class Plugin_Evacuation(WaypointPlugin):
    def __init__(self):
        ...

    def callback(self, data) -> OutputData:
        def calculate_forces(agent: pedsim_msgs.msg.AgentState) -> pedsim_msgs.msg.AgentFeedback:
            '''
            There are different adjustments you can make to the Simulation.

            room:
            There are different rooms to choose from:
                -"square": a standart square room with one exit
                -"long_room": a standart rectangle room with one exit
                -"long_room_v2": a rectangle room with two exits. Half of the agents will go to each exit
                -"edu_11", "edu_1", "edu_room": square rooms with one exit and different walls inside the room

            You can choose the number of agents in the simulation with "num_individuals".

            "num_steps" is the duraten the simulation will runn (recommended:1000)

            "method" is the method of integration. You should use leap_frog even though it will often explode
            since more relaible methods of integration like ode45 and monto carlo take a lot a computational power.
            '''
            # calls the simulation with:
            # the number of agents that are currently in the simulation,
            # the number of force-calculation steps the simulation should go through (each callback should be 1 step?),
            # the method of Integrating (leap-frog was the GoTo solution in the original project),
            # room_size (TODO: figure out what value to put)
            # room_square (TODO: figure out what value to put)

            sim = Simulation(num_individuals=len(pedsim_msgs.msg.TrackedPersons.tracks), num_steps=1, method="leap_frog", room_size=20, room="square")
            sim.run()           
            return sim.forces
        return [pedsim_msgs.msg.AgentFeedback(unforce=True) for agent in data.agents]
