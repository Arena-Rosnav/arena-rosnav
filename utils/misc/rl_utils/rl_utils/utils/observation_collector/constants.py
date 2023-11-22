MAX_WAIT = 2  # in seconds
SLEEP = 0.2  # in seconds


class TOPICS:
    LASER = "scan"
    FULL_RANGE_LASER = "full_scan"
    ROBOT_STATE = "odom"
    GOAL = "subgoal"

    GLOBALPLAN = "global_plan"

    PEDSIM_STATES = "pedsim_simulator/simulated_agents"


class OBS_DICT_KEYS:
    LASER = "laser_scan"
    ROBOT_POSE = "robot_pose"
    GOAL = "goal_in_robot_frame"
    DISTANCE_TO_GOAL = "distance_to_goal"
    LAST_ACTION = "last_action"
    GLOBAL_PLAN = "global_plan"

    PEDSIM_POSES = "pedsim_poses"
    PEDSIM_TWISTS = "pedsim_twists"
    PEDSIM_TYPES = "pedsim_types"
