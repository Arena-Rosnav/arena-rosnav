#!/usr/bin/env python3
"""
This file is used to calculate from the simulation data, various metrics, such as
- did a collision occur
- how long did the robot take form start to goal
the metrics / evaluation data will be saved to be preproccesed in the next step
"""
import numpy as np
import pandas as pd
import os
import yaml
import argparse 
import rospkg
import json

from utils import Utils


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--dir", "-d")

    return parser.parse_args()


class Action:
    STOP = "STOP"
    ROTATE = "ROTATE"
    MOVE = "MOVE"


class DoneReason:
    TIMEOUT = "TIMEOUT"
    GOAL_REACHED = "GOAL_REACHED"
    COLLISION = "COLLISION"


class Config:
    TIMEOUT_TRESHOLD = 180e9
    MAX_COLLISIONS = 3


class Metrics:
    def __init__(self, dir):
        self.dir = dir

        self.robot_params = Metrics.get_robot_params(self.dir)

        episode = pd.read_csv(self.dir + "/episode.csv", converters={
            "data": lambda val: 0 if len(val) <= 0 else int(val) 
        })
        laserscan = pd.read_csv(self.dir + "/scan.csv", converters={
            "data": Utils.string_to_float_list
        })
        odom = pd.read_csv(self.dir + "/odom.csv", converters={
            "data": lambda col: json.loads(col.replace("'", "\""))
        })
        cmd_vel = pd.read_csv(self.dir + "/cmd_vel.csv", converters={
            "data": Utils.string_to_float_list
        })
        start_goal = pd.read_csv(self.dir + "/start_goal.csv", converters={
            "start": Utils.string_to_float_list,
            "goal": Utils.string_to_float_list
        })

        laserscan = laserscan.rename(columns={"data": "laserscan"})
        odom = odom.rename(columns={"data": "odom"})
        cmd_vel = cmd_vel.rename(columns={"data": "cmd_vel"})

        data = pd.concat([episode, laserscan, odom, cmd_vel, start_goal], axis=1, join="inner")
        data = data.loc[:,~data.columns.duplicated()].copy()

        i = 0

        episode_data = {}

        while True:
            current_episode = data[data["episode"] == i]

            if len(current_episode) <= 0:
                break
            
            episode_data[i] = self.analyze_episode(current_episode, i)
            i = i + 1

        data = pd.DataFrame(episode_data).transpose().set_index("episode")
        data.to_csv(os.path.join(dir, "metrics.csv"))

        pass

    def analyze_episode(self, episode, index):
        positions, velocities = [], []

        for odom in episode["odom"]:
            positions.append(np.array(odom["position"]))
            velocities.append(np.array(odom["velocity"]))

        curvature, normalized_curvature = self.get_curvature(np.array(positions))
        roughness = self.get_roughness(np.array(positions))

        vel_absolute = self.get_velocity_abs(velocities)
        acceleration = self.get_acceleration(vel_absolute)
        jerk = self.get_jerk(vel_absolute)

        collisions, collision_amount = self.get_collisions(
            episode["laserscan"],
            self.robot_params["robot_radius"]
        )

        path_length, path_length_per_step = self.get_path_length(positions)

        time = int(list(episode["time"])[-1] - list(episode["time"])[0])

        start_position = self.get_mean_position(episode, "start")
        goal_position = self.get_mean_position(episode, "goal")

        return {
            "curvature": Metrics.round_values(curvature),
            "normalized_curvature": Metrics.round_values(normalized_curvature),
            "roughness": Metrics.round_values(roughness),
            "path_length_values": Metrics.round_values(path_length_per_step),
            "path_length": path_length,
            "acceleration": Metrics.round_values(acceleration),
            "jerk": Metrics.round_values(jerk),
            "velocity": Metrics.round_values(vel_absolute),
            "collision_amount": collision_amount,
            "collisions": list(collisions),
            "path": [list(p) for p in positions],
            "angle_over_length": self.get_angle_over_length(path_length, positions),
            "action_type": list(self.get_action_type(episode["cmd_vel"])),
            ## Ros time in ns
            "time_diff": time,
            "time": list(map(int, episode["time"].tolist())),
            "episode": index,
            "result": self.get_success(time, collision_amount),
            "cmd_vel": list(map(list, episode["cmd_vel"].to_list())),
            "goal": goal_position,
            "start": start_position
        }

    def get_mean_position(self, episode, key):
        positions = episode[key].to_list()
        counter = {}

        for p in positions:
            hash = ":".join([str(pos) for pos in p])

            counter[hash] = counter.get(hash, 0) + 1

        sorted_positions = dict(sorted(counter.items(), key=lambda x: x))

        return [float(r) for r in list(sorted_positions.keys())[0].split(":")]

    def get_position_for_collision(self, collisions, positions):
        for i, collision in enumerate(collisions):
            collisions[i][2] = positions[collision[0]]

        return collisions

    def get_angle_over_length(self, path_length, positions):
        total_yaw = 0

        for i, yaw in enumerate(positions[:-1]):
            yaw = yaw[2]
            next_yaw = positions[i + 1][2]

            total_yaw += abs(next_yaw - yaw)

        return total_yaw / path_length

    def get_success(self, time, collisions):
        if time >= Config.TIMEOUT_TRESHOLD:
            return DoneReason.TIMEOUT

        if collisions >= Config.MAX_COLLISIONS:
            return DoneReason.COLLISION

        return DoneReason.GOAL_REACHED

    def get_path_length(self, positions):
        path_length = 0
        path_length_per_step = []

        for i, position in enumerate(positions[:-1]):
            next_position = positions[i + 1]

            step_path_length = np.linalg.norm(position - next_position)

            path_length_per_step.append(step_path_length)
            path_length += step_path_length

        return path_length, path_length_per_step
    
    def get_collisions(self, laser_scans, lower_bound):
        """
        Calculates the collisions. Therefore, 
        the laser scans is examinated and all values below a 
        specific range are marked as collision.

        Argument:
            - Array laser scans representing the scans over
            time
            - the lower bound for which a collisions are counted

        Returns tupel of:
            - Array of tuples with indexs and time in which
            a collision happened
        """
        collisions = []
        collisions_marker = []

        for i, scan in enumerate(laser_scans):

            is_collision = len(scan[scan <= lower_bound]) > 0

            collisions_marker.append(int(is_collision))
            
            if is_collision:
                collisions.append(i)

        collision_amount = 0

        for i, coll in enumerate(collisions_marker[1:]):
            prev_coll = collisions_marker[i]

            if coll - prev_coll > 0:
                collision_amount += 1

        return collisions, collision_amount

    def get_action_type(self, actions):
        action_type = []

        for action in actions:
            if sum(action) == 0:
                action_type.append(Action.STOP)
            elif action[0] == 0 and action[1] == 0:
                action_type.append(Action.ROTATE)
            else:
                action_type.append(Action.MOVE)

        return action_type

    def get_curvature(self, positions):
        """
        Calculates the curvature and the normalized curvature
        for all positions in the list

        Returns an array of tuples with (curvature, normalized_curvature)
        """
        curvature_list = []
        normalized_curvature = []

        for i, position in enumerate(positions[:-2]):
            first = position
            second = positions[i + 1]
            third = positions[i + 2]

            curvature, normalized = Metrics.calc_curvature(first, second, third)

            curvature_list.append(curvature)
            normalized_curvature.append(normalized)

        return curvature_list, normalized_curvature

    def get_roughness(self, positions):
        roughness_list = []

        for i, position in enumerate(positions[:-2]):
            first = position
            second = positions[i + 1]
            third = positions[i + 2]

            roughness_list.append(Metrics.calc_roughness(first, second, third))

        return roughness_list

    def get_velocity_abs(self, velocities):
        return [(i ** 2 + j ** 2) ** 0.5 for i, j, z in velocities]

    def get_acceleration(self, vel_abs):
        acc_list = []

        for i, vel in enumerate(vel_abs[:-1]):
            acc_list.append(vel_abs[i + 1] - vel)

        return acc_list

    def get_jerk(self, vel_abs):
        """
        jerk is the rate at which an objects acceleration changes with respect to time
        """
        jerk_list = []

        for i, velocity in enumerate(vel_abs[:-2]):
            first = velocity
            second = vel_abs[i + 1]
            third = vel_abs[i + 2]

            jerk = Metrics.calc_jerk(first, second, third)

            jerk_list.append(jerk)

        return jerk_list

    @staticmethod
    def calc_curvature(first, second, third):
        triangle_area = Metrics.calc_triangle_area(first, second, third)

        divisor = (
            np.abs(np.linalg.norm(first - second)) 
            * np.abs(np.linalg.norm(second - third))
            * np.abs(np.linalg.norm(third - first))
        )

        if divisor == 0:
            return 0, 0

        curvature = 4 * triangle_area / divisor

        normalized = (
            curvature * (
                np.abs(np.linalg.norm(first - second)) 
                + np.abs(np.linalg.norm(second - third))
            )
        )

        return curvature, normalized

    @staticmethod
    def round_values(values, digits=3):
        return [round(v, digits) for v in values]

    @staticmethod
    def calc_roughness(first, second, third):
        triangle_area = Metrics.calc_triangle_area(first, second, third)

        return 2 * triangle_area / np.abs(np.linalg.norm(third - first)) ** 2

    @staticmethod
    def calc_jerk(first, second, third):
        a1 = second - first
        a2 = third - second

        jerk = np.abs(a2 - a1)

        return jerk

    @staticmethod
    def calc_triangle_area(first, second, third):
        return (
            0.5 * np.abs(
                first[0] * (second[1] - third[1]) 
                + second[0] * (third[1] - first[1]) 
                + third[0] * (first[1] - second[1])
            )
        )

    @staticmethod
    def get_robot_params(dir):
        with open(os.path.join(dir, "params.yaml")) as file:
            content = yaml.safe_load(file)

            model = content["model"]

        robot_model_params_file = os.path.join(
            rospkg.RosPack().get_path("arena-simulation-setup"), 
            "robot", 
            model, 
            "model_params.yaml"
        )

        with open(robot_model_params_file, "r") as file:
            return yaml.safe_load(file)


if __name__ == "__main__":
    arguments = parse_args()

    Metrics(arguments.dir)
