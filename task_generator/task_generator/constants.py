class Constants:
    GOAL_REACHED_TOLERANCE = 1.0
    TIMEOUT = 3.0 * 60 ## 3 min
    WAIT_FOR_SERVICE_TIMEOUT = 5 # 5 secs
    MAX_RESET_FAIL_TIMES = 3

    class ObstacleManager:
        DYNAMIC_OBSTACLES = 0
        STATIC_OBSTACLES = 0

        OBSTACLE_MAX_RADIUS = 0.6

        OBSTACLE_MAX_RADIUS = 0.6
    
    class RobotManager:
        SPAWN_ROBOT_SAFE_DIST = 0.4

    class Environment:
        FLATLAND = "flatland"
        GAZEBO = "gazebo"

    class ArenaType:
        TRAINING = "training"
        DEPLOYMENT = "deployment"


class TaskMode:
    RANDOM = "random"
    STAGED = "staged"
    SCENARIO = "scenario"

    class Random:
        MIN_DYNAMIC_OBS = 10
        MAX_DYNAMIC_OBS = 15
        MIN_STATIC_OBS = 10
        MAX_STATIC_OBS = 20

    class Scenario:
        RESETS_DEFAULT = 5

class FlatlandRandomModel:
    BODY = {
        "name": "base_link",
        "pose": [0, 0, 0],
        "color": [1, 0.2, 0.1, 1.0],
        "footprints": []
    }
    FOOTPRINT = {
        "density": 1,
        "restitution": 1,
        "layers": ["all"],
        "collision": "true",
        "sensor": "false"
    }
    MIN_RADIUS = 0.2
    MAX_RADIUS = 0.6
    RANDOM_MOVE_PLUGIN = {
        "type": "RandomMove",
        "name": "RandomMove_Plugin",
        "body": "base_link"
    }
    LINEAR_VEL = 0.2
    ANGLUAR_VEL_MAX = 0.2

class Pedsim:
    START_UP_MODE = "default"
    WAIT_TIME = 0.0
    TRIGGER_ZONE_RADIUS = 0.0
    CHATTING_PROBABILITY = 0.0
    TELL_STORY_PROBABILITY = 0.0
    GROUP_TALKING_PROBABILITY = 0.0
    TALKING_AND_WALKING_PROBABILITY = 0.0
    REQUESTING_SERVICE_PROBABILITY = 0.0
    REQUESTING_GUIDE_PROBABILITY = 0.0
    REQUESTING_FOLLOWER_PROBABILITY = 0.0
    MAX_TALKING_DISTANCE = 5.0
    MAX_SERVICING_RADIUS = 5.0
    TALKING_BASE_TIME = 10.0
    TELL_STORY_BASE_TIME = 0.0
    GROUP_TALKING_BASE_TIME = 10.0
    TALKING_AND_WALKING_BASE_TIME = 6.0
    RECEIVING_SERVICE_BASE_TIME = 20.0
    REQUESTING_SERVICE_BASE_TIME = 30.0
    FORCE_FACTOR_DESIRED = 1.0
    FORCE_FACTOR_OBSTACLE = 1.0
    FORCE_FACTOR_SOCIAL = 5.0
    FORCE_FACTOR_ROBOT = 0.0
    WAYPOINT_MODE = 0