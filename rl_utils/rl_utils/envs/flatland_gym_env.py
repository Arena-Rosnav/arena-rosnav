#! /usr/bin/env python3
import time
import math
import gym
import os
from stable_baselines3.common.env_checker import check_env

import numpy as np
import rospy
import time
from geometry_msgs.msg import Twist
from flatland_msgs.msg import StepWorld
from std_srvs.srv import Empty

from task_generator.tasks.utils import get_predefined_task

from ..utils.reward import RewardCalculator
from ..utils.observation_collector import ObservationCollector
from rosnav.rosnav_space_manager.rosnav_space_manager import RosnavSpaceManager


class FlatlandEnv(gym.Env):
    """Custom Environment that follows gym interface"""

    def __init__(
        self,
        ns: str,
        reward_fnc: str,
        is_action_space_discrete,
        safe_dist: float = None,
        goal_radius: float = 0.1,
        max_steps_per_episode=100,
        task_mode: str = "staged",
        PATHS: dict = dict(),
        extended_eval: bool = False,
        *args,
        **kwargs,
    ):
        """Default env
        Flatland yaml node check the entries in the yaml file, therefore other robot related parameters cound only be saved in an other file.
        TODO : write an uniform yaml paser node to handel with multiple yaml files.


        Args:
            task (ABSTask): [description]
            reward_fnc (str): [description]
            train_mode (bool): bool to differ between train and eval env during training
            is_action_space_discrete (bool): [description]
            safe_dist (float, optional): [description]. Defaults to None.
            goal_radius (float, optional): [description]. Defaults to 0.1.
            extended_eval (bool): more episode info provided, no reset when crashing
        """
        super(FlatlandEnv, self).__init__()

        self.ns = ns
        try:
            # given every environment enough time to initialize, if we dont put sleep,
            # the training script may crash.
            import re

            ns_int = int(re.search(r"\d+", ns)[0])
            time.sleep((ns_int + 1) * 2)
        except Exception:
            rospy.logwarn(
                "Can't not determinate the number of the environment, training script may crash!"
            )
            time.sleep(2)

        # process specific namespace in ros system
        self.ns_prefix = lambda x: os.path.join(self.ns, x)

        if not rospy.get_param("/debug_mode"):
            rospy.init_node("env_" + self.ns, anonymous=True)

        self._extended_eval = extended_eval
        self._is_train_mode = rospy.get_param("/train_mode")
        self._is_action_space_discrete = is_action_space_discrete

        self.model_space_encoder = RosnavSpaceManager()

        # observation collector
        self.observation_collector = ObservationCollector(
            self.ns,
            self.model_space_encoder._laser_num_beams,
            external_time_sync=False,
        )

        self.action_space = self.model_space_encoder.get_action_space()
        self.observation_space = self.model_space_encoder.get_observation_space()

        # reward calculator
        if safe_dist is None:
            safe_dist = self.model_space_encoder._radius + 0.1

        self.reward_calculator = RewardCalculator(
            holonomic=self.model_space_encoder._is_holonomic,
            robot_radius=self.model_space_encoder._radius,
            safe_dist=safe_dist,
            goal_radius=goal_radius,
            rule=reward_fnc,
            extended_eval=self._extended_eval,
        )

        # action agent publisher
        if self._is_train_mode:
            self.agent_action_pub = rospy.Publisher(
                self.ns_prefix("cmd_vel"), Twist, queue_size=1
            )
        else:
            self.agent_action_pub = rospy.Publisher(
                self.ns_prefix("cmd_vel_pub"), Twist, queue_size=1
            )

        # service clients
        if self._is_train_mode:
            self._service_name_step = self.ns_prefix("step_world")
            # self._sim_step_client = rospy.ServiceProxy(self._service_name_step, StepWorld)
            self._step_world_publisher = rospy.Publisher(
                self._service_name_step, StepWorld, queue_size=10
            )
            self._step_world_srv = rospy.ServiceProxy(
                self._service_name_step, Empty, persistent=True
            )

        # instantiate task manager
        self.task = get_predefined_task(
            self.ns,
            mode=task_mode,
            start_stage=kwargs["curr_stage"],
            paths=PATHS,
        )

        self._steps_curr_episode = 0
        self._episode = 0
        self._max_steps_per_episode = max_steps_per_episode
        self._last_action = np.array([0, 0, 0])  # linear x, linear y, angular z

        # for extended eval
        self._action_frequency = 1 / rospy.get_param("/robot_action_rate", 10)
        self._last_robot_pose = None
        self._distance_travelled = 0
        self._safe_dist_counter = 0
        self._collisions = 0
        self._in_crash = False

        self.last_mean_reward = 0
        self.mean_reward = [0, 0]

        self.step_time = [0, 0]

        self._done_reasons = {
            "0": "Timeout",
            "1": "Crash",
            "2": "Success",
        }
        self._done_hist = 3 * [0]

    def _pub_action(self, action: np.ndarray) -> Twist:
        assert len(action) == 3

        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.linear.y = action[1]
        action_msg.angular.z = action[2]

        self.agent_action_pub.publish(action_msg)

    def step(self, action: np.ndarray):
        """
        done_reasons:   0   -   exceeded max steps
                        1   -   collision with obstacle
                        2   -   goal reached
        """

        start_time = time.time()

        decoded_action = self.model_space_encoder.decode_action(action)
        self._pub_action(decoded_action)

        self._pub_action(decoded_action)

        if self._is_train_mode:
            self.call_service_takeSimStep()

        obs_dict = self.observation_collector.get_observations(
            last_action=self._last_action
        )

        self._steps_curr_episode += 1

        obs_dict = self.observation_collector.get_observations(
            last_action=self._last_action
        )
        self._last_action = decoded_action

        # calculate reward
        reward, reward_info = self.reward_calculator.get_reward(
            laser_scan=obs_dict["laser_scan"],
            goal_in_robot_frame=obs_dict["goal_in_robot_frame"],
            action=decoded_action,
            global_plan=obs_dict["global_plan"],
            robot_pose=obs_dict["robot_pose"],
        )
        done = reward_info["is_done"]

        # extended eval info
        if self._extended_eval:
            self._update_eval_statistics(obs_dict, reward_info)

        # info
        info = {}

        if done:
            info["done_reason"] = reward_info["done_reason"]
            info["is_success"] = reward_info["is_success"]

        if self._steps_curr_episode >= self._max_steps_per_episode:
            done = True
            info["done_reason"] = 0
            info["is_success"] = 0

        # for logging
        if self._extended_eval and done:
            info["collisions"] = self._collisions
            info["distance_travelled"] = round(self._distance_travelled, 2)
            info["time_safe_dist"] = self._safe_dist_counter * self._action_frequency
            info["time"] = self._steps_curr_episode * self._action_frequency

        self.step_time[1] += 1
        self.mean_reward[1] += 1
        self.mean_reward[0] += reward

        if done:
            # print(self.ns_prefix, "DONE", info["done_reason"], sum(self._done_hist), min(obs_dict["laser_scan"]))

            # print(f"[{self.ns_prefix}]:\t\t", self.step_time[0] / self.step_time[1])

            # if self._steps_curr_episode <= 1:
            #     print(self.ns_prefix, "DONE", info, min(obs_dict["laser_scan"]), obs_dict["goal_in_robot_frame"])

            if sum(self._done_hist) >= 5:
                mean_reward = self.mean_reward[0] / self.mean_reward[1]
                diff = round(mean_reward - self.last_mean_reward, 5)

                print(
                    f"[{self.ns}] Last 5 Episodes:\t"
                    f"{self._done_reasons[str(0)]}: {self._done_hist[0]}\t"
                    f"{self._done_reasons[str(1)]}: {self._done_hist[1]}\t"
                    f"{self._done_reasons[str(2)]}: {self._done_hist[2]}\t"
                    f"Mean step time: {round(self.step_time[0] / self.step_time[1] * 100, 2)}\t"
                    f"Mean reward: {round(mean_reward, 5)} ({'+' if diff >= 0 else ''}{diff})"
                )
                self._done_hist = [0] * 3
                self.step_time = [0, 0]
                self.last_mean_reward = mean_reward
                self.mean_reward = [0, 0]
            self._done_hist[int(info["done_reason"])] += 1

        self.step_time[0] += time.time() - start_time

        return (
            self.model_space_encoder.encode_observation(
                obs_dict, ["laser_scan", "goal_in_robot_frame", "last_action"]
            ),
            reward,
            done,
            info,
        )

    def call_service_takeSimStep(self, t=None):
        request = StepWorld()
        request.required_time = 0 if t == None else t

        self._step_world_srv()

        # self._step_world_publisher.publish(request)

    def reset(self):

        # set task
        # regenerate start position end goal position of the robot and change the obstacles accordingly
        self._episode += 1
        self.agent_action_pub.publish(Twist())
        self.task.reset()
        self.reward_calculator.reset()
        self._steps_curr_episode = 0
        self._last_action = np.array([0, 0, 0])

        if self._is_train_mode:
            self.call_service_takeSimStep()

        # extended eval info
        if self._extended_eval:
            self._last_robot_pose = None
            self._distance_travelled = 0
            self._safe_dist_counter = 0
            self._collisions = 0

        obs_dict = self.observation_collector.get_observations()
        return self.model_space_encoder.encode_observation(
            obs_dict, ["laser_scan", "goal_in_robot_frame", "last_action"]
        )  # reward, done, info can't be included

    def close(self):
        pass

    def _update_eval_statistics(self, obs_dict: dict, reward_info: dict):
        """
        Updates the metrics for extended eval mode

        param obs_dict (dict): observation dictionary from ObservationCollector.get_observations(),
            necessary entries: 'robot_pose'
        param reward_info (dict): dictionary containing information returned from RewardCalculator.get_reward(),
            necessary entries: 'crash', 'safe_dist'
        """
        # distance travelled
        if self._last_robot_pose is not None:
            self._distance_travelled += FlatlandEnv.get_distance(
                self._last_robot_pose, obs_dict["robot_pose"]
            )

        # collision detector
        if "crash" in reward_info:
            if reward_info["crash"] and not self._in_crash:
                self._collisions += 1
                # when crash occures, robot strikes obst for a few consecutive timesteps
                # we want to count it as only one collision
                self._in_crash = True
        else:
            self._in_crash = False

        # safe dist detector
        if "safe_dist" in reward_info and reward_info["safe_dist"]:
            self._safe_dist_counter += 1

        self._last_robot_pose = obs_dict["robot_pose"]

    @staticmethod
    def get_distance(pose_1, pose_2):
        return math.hypot(pose_2.x - pose_1.x, pose_2.y - pose_1.y)


if __name__ == "__main__":
    rospy.init_node("flatland_gym_env", anonymous=True, disable_signals=False)
    print("start")

    flatland_env = FlatlandEnv()
    rospy.loginfo("======================================================")
    rospy.loginfo("CSVWriter initialized.")
    rospy.loginfo("======================================================")
    check_env(flatland_env, warn=True)

    # init env
    obs = flatland_env.reset()

    # run model
    n_steps = 200
    for _ in range(n_steps):
        # action, _states = model.predict(obs)
        action = flatland_env.action_space.sample()

        obs, rewards, done, info = flatland_env.step(action)

        time.sleep(0.1)
