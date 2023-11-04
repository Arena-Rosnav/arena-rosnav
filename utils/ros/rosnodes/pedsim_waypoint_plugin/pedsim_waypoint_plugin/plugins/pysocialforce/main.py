from pedsim_waypoint_plugin.pedsim_waypoint_generator import OutputData, PedsimWaypointGenerator, InputData, WaypointPluginName, WaypointPlugin
import pedsim_msgs.msg

import numpy as np
from pathlib import Path
from typing import Dict, List, Optional, Type, TypeVar
import random

import sys
sys.path.append(str(Path(__file__).resolve().parent))
import pysocialforce as psf


@PedsimWaypointGenerator.register(WaypointPluginName.PYSOCIALFORCE)
class Plugin_PySocialForce(WaypointPlugin):
    def __init__(self):
        self.first_call = True
        self.simulator = None
        self.groups = list()

    def assign_groups(self,
                   agents: dict,
                   groups: List[pedsim_msgs.msg.AgentGroup],
                   ) -> list:
        
        # assign new agents to groups
        for agent_id, idx in agents.items():
            if agent_id in self.groups:
                # agent already went through assignment
                pass

            assigned_group = random.randint(0, len(self.groups))
            if assigned_group == len(self.groups):
                self.groups.append([agent_id])
            else:
                self.groups[assigned_group].append(agent_id)

        # form group list
        return [[agents[id] for id in group] for group in self.groups]

    @classmethod
    def get_state_data(cls, 
                       agents: List[pedsim_msgs.msg.AgentState], 
                       groups: List[pedsim_msgs.msg.AgentGroup],
                       reset_velocity: bool = False
                       ) -> np.ndarray:
        
        idx_assignment = dict()
        state_data = list()
        for idx in range(len(agents)):
            agent = agents[idx]

            p_x = agent.pose.position.x
            p_y = agent.pose.position.y
            v_x = agent.twist.linear.x if not reset_velocity else 1.0
            v_y = agent.twist.linear.y if not reset_velocity else 1.0
            d_x = agent.destination.x
            d_y = agent.destination.y
            state_data.append([p_x, p_y, v_x, v_y, d_x, d_y])
            idx_assignment[agent.id] = idx
        state = np.array(state_data)
        
        return state, idx_assignment
    
    @classmethod
    def map_force_to_feedback(cls,
                              agents: List[pedsim_msgs.msg.AgentState],
                              force: np.ndarray
                              ) -> List[pedsim_msgs.msg.AgentFeedback]:
        feedbacks = list()
        for i in range(force.shape[0]):
            feedback = pedsim_msgs.msg.AgentFeedback()
            feedback.id = agents[i].id
            feedback.force.x = force[i, 0]
            feedback.force.y = force[i, 1]

            feedbacks.append(feedback)
        
        return feedbacks


    def callback(self, data: InputData) -> OutputData:
        if len(data.agents) < 1:
            return list()
        
        state, agent_idx = self.get_state_data(data.agents, data.groups, reset_velocity=self.first_call)
        groups = self.assign_groups(agent_idx, data.groups)
        print(groups)

        if self.first_call:
            # instantiate sim
            self.first_call = False
            # TODO: hard-coded -> remove once map edges are in line obstacles
            obs = [[0.0, 25.0, 0.5, 0.5], [0.0, 25.0, 20.5, 20.5], [0.5, 0.5, 0.0, 21.0], [24.5, 24.5, 0.0, 21.0]]
            self.simulator = psf.Simulator(
                state=state,
                groups=groups,
                obstacles=obs,
                config_file=Path(__file__).resolve().parent.joinpath("pysocialforce/config/default.toml")
            )
        else:
            # update sim
            self.simulator.peds.update(state, groups)

        forces = self.simulator.compute_forces()

        return self.map_force_to_feedback(data.agents, forces)
