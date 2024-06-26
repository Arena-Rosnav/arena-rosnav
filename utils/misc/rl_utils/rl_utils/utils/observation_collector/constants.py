import enum
from crowdsim_agents.utils import SemanticAttribute

MAX_WAIT = 5  # in seconds
SLEEP = 0.05  # in seconds


class DONE_REASONS(enum.Enum):
    STEP_LIMIT = 0
    COLLISION = 1
    SUCCESS = 2


class TOPICS:
    LASER = "scan"
    FULL_RANGE_LASER = "full_scan"
    ROBOT_STATE = "odom"
    GOAL = "move_base_simple/goal"
    SUBGOAL = "subgoal"

    GLOBALPLAN = "global_plan"

    PEDSIM_STATES = "pedsim_simulator/simulated_agents"
    PEDSIM_SEMANTIC = "crowdsim_agents/semantic"

    PED_SAFE_DIST = "ped_safe_dist"
    OBS_SAFE_DIST = "obs_safe_dist"
    COLLISION = "collision"
    IMAGE_DEPTH = "rgbd/depth"
    IMAGE_COLOR = "rgbd/image"


class OBS_DICT_KEYS:
    """
    Constants for observation dictionary keys.
    """

    LASER = "laser_scan"  # check
    FULL_RANGE_LASER = "full_range_laser_scan"
    ROBOT_POSE = "robot_pose"  # check
    SUBGOAL = "subgoal"  # check
    GOAL = "goal"  # check
    GOAL_DIST_ANGLE = "goal_in_robot_frame"  #
    GOAL_LOCATION_IN_ROBOT_FRAME = "goal_location_in_robot_frame"  #
    GOAL_LOCATION = "goal_location"  #
    DISTANCE_TO_GOAL = "distance_to_goal"  #
    LAST_ACTION = "last_action"  #
    GLOBAL_PLAN = "global_plan"  #
    INTER_REPLAN = "inter_replan"
    DONE = "is_done"  #

    PED_SAFE_DIST = "ped_safe_dist"  #
    OBS_SAFE_DIST = "obs_safe_dist"  #
    COLLSION = "collision"  #
    IMAGE_DEPTH = "image_depth"
    IMAGE_COLOR = "image_color"

    # why not have SEMANTIC = SemanticAttribute ?
    class SEMANTIC(enum.Enum):
        PEDESTRIAN_LOCATION = SemanticAttribute.IS_PEDESTRIAN.value
        PEDESTRIAN_TYPE = SemanticAttribute.PEDESTRIAN_TYPE.value
        PEDESTRIAN_MOVING = SemanticAttribute.IS_PEDESTRIAN_MOVING.value
        PEDESTRIAN_VEL_X = SemanticAttribute.PEDESTRIAN_VEL_X.value
        PEDESTRIAN_VEL_Y = SemanticAttribute.PEDESTRIAN_VEL_Y.value
        PEDESTRIAN_SOCIAL_STATE = SemanticAttribute.SOCIAL_STATE.value

        RELATIVE_LOCATION = "relative_location"
        RELATIVE_X_VEL = "relative_x_vel"
        RELATIVE_Y_VEL = "relative_y_vel"

        MIN_DISTANCE_PER_TYPE = "min_distance_per_type"
