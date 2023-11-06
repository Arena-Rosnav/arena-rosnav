from pedsim_waypoint_plugin.pedsim_waypoint_generator import OutputData, PedsimWaypointGenerator, InputData, WaypointPluginName, WaypointPlugin
import pedsim_msgs.msg

import numpy as np
from pathlib import Path
from typing import Dict, List, Optional, Type, TypeVar
import random

import sys
sys.path.append(str(Path(__file__).resolve().parent))
import pysocialforce as psf


@PedsimWaypointGenerator.register(WaypointPluginName.PYSOCIAL1)
class Plugin_PySocialForce(WaypointPlugin):
    def __init__(self):
        self.first_call = True
        self.simulator = None
        self.groups = dict()
        self.group_count = 0

    def assign_groups(self,
                   agents: dict,
                   _groups: List[pedsim_msgs.msg.AgentGroup],
                   ) -> list:
        
        # assign new agents to groups
        for agent_id, idx in agents.items():
            if agent_id in self.groups:
                # agent already went through assignment
                continue

            # pick between existing groups and a new single one
            assigned_group = random.randint(0, self.group_count)
            if assigned_group == self.group_count:
                self.group_count += 1
            self.groups[agent_id] = assigned_group

        # form group list
        new_groups = [list() for _ in range(self.group_count)]
        for agent_id, group_idx in self.groups.items():
            new_groups[group_idx].append(agents[agent_id])

        return new_groups
    
    def overwrite_group_dest(self,
                        state: np.ndarray,
                        groups: List,
                        ) -> np.ndarray:
        # TODO
        for group in groups:
            leader = group[0]

            for i in range(1, len(group)):
                member = group[i]
                state[member, 4] = state[leader, 4]
                state[member, 5] = state[leader, 5]

        return state

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
        state = self.overwrite_group_dest(state, groups)
        print(state[:,4:6])
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
