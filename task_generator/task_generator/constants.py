class Constants:
    GOAL_REACHED_TOLERANCE = 1.0
    TIMEOUT = 3.0 * 60  ## 3 min
    WAIT_FOR_SERVICE_TIMEOUT = 60  # 5 secs
    MAX_RESET_FAIL_TIMES = 10

    class ObstacleManager:
        DYNAMIC_OBSTACLES = 0
        STATIC_OBSTACLES = 0

        OBSTACLE_MAX_RADIUS = 0.6

        OBSTACLE_MAX_RADIUS = 0.6

    class RobotManager:
        SPAWN_ROBOT_SAFE_DIST = 0.1

    class Simulator:
        FLATLAND = "flatland"
        GAZEBO = "gazebo"

    class ArenaType:
        TRAINING = "training"
        DEPLOYMENT = "deployment"

    class MapGenerator:
        NODE_NAME = "map_generator"
        MAP_FOLDER_NAME = "dynamic_map"

    PLUGIN_FULL_RANGE_LASER = {
        "type": "Laser",
        "name": "full_static_laser",
        "frame": "full_laser",
        "topic": "full_scan",
        "body": "base_link",
        "broadcast_tf": "true",
        "origin": [0, 0, 0],
        "range": 2.0,
        "angle": {"min": -3.14, "max": 3.14, "increment": 0.01745},
        "noise_std_dev": 0.0,
        "update_rate": 10,
    }


class TaskMode:
    RANDOM = "random"
    STAGED = "staged"
    SCENARIO = "scenario"
    DYNAMIC_MAP_RANDOM = "dynamic_map_random"
    DYNAMIC_MAP_STAGED = "dynamic_map_staged"

    class Random:
        MIN_DYNAMIC_OBS = 0
        MAX_DYNAMIC_OBS = 0
        MIN_STATIC_OBS = 0
        MAX_STATIC_OBS = 0

    class Scenario:
        RESETS_DEFAULT = 5


class FlatlandRandomModel:
    BODY = {
        "name": "base_link",
        "pose": [0, 0, 0],
        "color": [1, 0.2, 0.1, 1.0],
        "footprints": [],
    }
    FOOTPRINT = {
        "density": 1,
        "restitution": 1,
        "layers": ["all"],
        "collision": "true",
        "sensor": "false",
    }
    MIN_RADIUS = 0.2
    MAX_RADIUS = 0.6
    RANDOM_MOVE_PLUGIN = {
        "type": "RandomMove",
        "name": "RandomMove_Plugin",
        "body": "base_link",
    }
    LINEAR_VEL = 0.2
    ANGLUAR_VEL_MAX = 0.2


class Pedsim:
    VMAX = 0.3
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
