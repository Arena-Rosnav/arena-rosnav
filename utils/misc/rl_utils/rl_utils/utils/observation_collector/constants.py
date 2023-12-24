import enum
from pedsim_agents.utils import SemanticAttribute


class TOPICS:
    LASER = "scan"
    FULL_RANGE_LASER = "full_scan"
    ROBOT_STATE = "odom"
    GOAL = "goal"

    GLOBALPLAN = "global_plan"

    PEDSIM_STATES = "pedsim_simulator/simulated_agents"
    PEDSIM_SEMANTIC = "pedsim_agents/semantic"


class OBS_DICT_KEYS:
    """
    Constants for observation dictionary keys.
    """

    LASER = "laser_scan"
    ROBOT_POSE = "robot_pose"
    GOAL = "goal_in_robot_frame"
    DISTANCE_TO_GOAL = "distance_to_goal"
    LAST_ACTION = "last_action"
    GLOBAL_PLAN = "global_plan"
    DONE = "is_done"

    # why not have SEMANTIC = SemanticAttribute ?
    class SEMANTIC(enum.Enum):
        PEDESTRIAN_LOCATION = SemanticAttribute.IS_PEDESTRIAN.value
        PEDESTRIAN_TYPE = SemanticAttribute.PEDESTRIAN_TYPE.value
        PEDESTRIAN_MOVING = SemanticAttribute.IS_PEDESTRIAN_MOVING.value
        PEDESTRIAN_VEL_X = SemanticAttribute.PEDESTRIAN_VEL_X.value
        PEDESTRIAN_VEL_Y = SemanticAttribute.PEDESTRIAN_VEL_Y.value
