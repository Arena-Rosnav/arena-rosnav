from pedsim_waypoint_plugin.pedsim_waypoint_generator import OutputData, PedsimWaypointGenerator, InputData, WaypointPluginName, WaypointPlugin
import pedsim_msgs.msg

import numpy as np
from pathlib import Path
from typing import Dict, List, Optional, Type, TypeVar
import random
import rospy

import sys
sys.path.append(str(Path(__file__).resolve().parent))
import pysocialforce as psf


@PedsimWaypointGenerator.register(WaypointPluginName.PYSOCIAL1)
class Plugin_PySocialForce(WaypointPlugin):
    FACTOR = 0.3
    GROUP_DEST_DIST = 0.5

    def __init__(self):
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

        for group in groups:
            leader = group[0]
            
            for i in range(1, len(group)):
                member = group[i]
                
                row = (i // 8) + 1
                x_off = (1 if (i + 1) % 8 < 3 else (-1 if 3 < (i + 1) % 8 < 7 else 0)) * row * self.GROUP_DEST_DIST
                y_off = (1 if i % 8 > 4 else (-1 if 0 < i % 8 < 4 else 0)) * row * self.GROUP_DEST_DIST
                state[member, 4] = state[leader, 4] + x_off
                state[member, 5] = state[leader, 5] + y_off

        return state

    @staticmethod
    def get_state_data(agents: List[pedsim_msgs.msg.AgentState], 
                       groups: List[pedsim_msgs.msg.AgentGroup]
                       ) -> np.ndarray:
        
        idx_assignment = dict()
        state_data = list()
        for idx in range(len(agents)):
            agent = agents[idx]

            p_x = agent.pose.position.x
            p_y = agent.pose.position.y
            v_x = agent.twist.linear.x
            v_y = agent.twist.linear.y
            d_x = agent.destination.x
            d_y = agent.destination.y
            state_data.append([p_x, p_y, v_x, v_y, d_x, d_y])
            idx_assignment[agent.id] = idx
        state = np.array(state_data)
        
        return state, idx_assignment
    
    @staticmethod
    def map_force_to_feedback(agents: List[pedsim_msgs.msg.AgentState],
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
    
    @staticmethod
    def extract_obstacles(obstacles: List[pedsim_msgs.msg.LineObstacle]
                          ) -> List[List[int]]:
        obs_list = list()

        for obs in obstacles:
            x_min = min(obs.start.x, obs.end.x)
            x_max = max(obs.start.x, obs.end.x)
            y_min = min(obs.start.y, obs.end.y)
            y_max = max(obs.start.y, obs.end.y)

            obs_list.append([x_min, x_max, y_min, y_max])

        return obs_list


    def callback(self, data: InputData) -> OutputData:
        if len(data.agents) < 1:
            return list()
        
        state, agent_idx = self.get_state_data(data.agents, data.groups)
        groups = self.assign_groups(agent_idx, data.groups)
        state = self.overwrite_group_dest(state, groups)
        obs = self.extract_obstacles(data.line_obstacles)

        rospy.logdebug("Assigned Groups: " + groups.__str__())

        simulator = psf.Simulator(
            state=state,
            groups=groups,
            obstacles=obs,
            config_file=Path(__file__).resolve().parent.joinpath("pysocialforce/config/default.toml")
        )

        forces = self.FACTOR * simulator.compute_forces()

        return self.map_force_to_feedback(data.agents, forces)
