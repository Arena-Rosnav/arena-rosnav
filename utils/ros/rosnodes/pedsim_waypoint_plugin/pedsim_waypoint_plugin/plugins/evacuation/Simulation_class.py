import numpy as np
import sys

from diff_equation import Diff_Equ

from Room import Room

import Integrators

'''
class to put the hole thing together. Its main parts
are the Differential Equation, the rooms, and the method 
of integration. The Integration can be done by calling
the run function. If the integration is done the results 
are saved in self.y for later use for example to display
them with with the function "Show"'''

class Simulation:
    def __init__(self, num_individuals, num_steps, method="leap_frog", tau=0.1, v_des=1.5, room="square",
                 room_size=25):

        std_deviation = 0.07                    
        variation = np.random.normal(loc=1, scale=std_deviation, size=(1, num_individuals)) # is late used to make the agents differ in weight and size

        # Constants
        self.L = room_size  # size of square room (m)
        self.N = num_individuals  # quantity of pedestrians
        self.tau = tau  # time-step (s)
        self.num_steps = num_steps  # number of steps for integration

        # Agent information
        self.radii = 0.4 * (np.ones(self.N)*variation).squeeze()  # radii of pedestrians (m)
        self.v_des = v_des * np.ones(self.N)  # desired velocity (m/s)
        self.m = 80 * (np.ones(self.N)*variation).squeeze()  # mass of pedestrians (kg)
        self.forces = None              # forces on the agents
        self.agents_escaped = None    #number of agents escaped by timesteps
        self.v = np.zeros((2, self.N, self.num_steps))  # Three dimensional array of velocity
        self.y = np.zeros(
            (2, self.N, self.num_steps))  # Three dimensional array of place: x = coordinates, y = Agent, z=Time
        
        # other
        self.room = Room(room, room_size)  # kind of room the simulation runs in
        self.method = getattr(Integrators, method)  # method used for integration
        self.diff_equ = Diff_Equ(self.N, self.L, self.tau, self.room, self.radii, self.m)  # initialize Differential equation

    # calls the method of integration with the starting positions, diffequatial equation, number of steps, and delta t = tau
    def run(self):
        self.y, self.agents_escaped, self.forces = self.method(self.y[:, :, 0], self.v[:, :, 0], self.diff_equ.f, self.num_steps, self.tau, self.room)