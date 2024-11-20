
from enum import Enum
import enum
import typing
from task_generator.shared import Namespace


class Constants:

    DEFAULT_PEDESTRIAN_MODEL = "actor1"

    TASK_GENERATOR_SERVER_NODE = Namespace("task_generator_server")

    class Simulator(Enum):
        DUMMY = "dummy"
        FLATLAND = "flatland"
        GAZEBO = "gazebo"
        UNITY = "unity"

    class ArenaType(Enum):
        TRAINING = "training"
        DEPLOYMENT = "deployment"

    class EntityManager(Enum):
        DUMMY = "dummy"
        PEDSIM = "pedsim"
        CROWDSIM = "crowdsim"

    class TaskMode:
        @enum.unique
        class TM_Obstacles(enum.Enum):
            PARAMETRIZED = "parametrized"
            RANDOM = "random"
            SCENARIO = "scenario"

            @classmethod
            def prefix(cls, *args):
                return Namespace("tm_obstacles")(*args)

            @classmethod
            def default(cls) -> "Constants.TaskMode.TM_Obstacles":
                return cls.RANDOM

        @enum.unique
        class TM_Robots(enum.Enum):
            GUIDED = "guided"
            EXPLORE = "explore"
            RANDOM = "random"
            SCENARIO = "scenario"

            @classmethod
            def prefix(cls, *args):
                return Namespace("tm_robots")(*args)

            @classmethod
            def default(cls) -> "Constants.TaskMode.TM_Robots":
                return cls.RANDOM

        @enum.unique
        class TM_Module(enum.Enum):
            STAGED = "staged"
            DYNAMIC_MAP = "dynamic_map"
            CLEAR_FORBIDDEN_ZONES = "clear_forbidden_zones"
            RVIZ_UI = "rviz_ui"
            BENCHMARK = "benchmark"

            @classmethod
            def prefix(cls, *args):
                return Namespace("tm_module")(*args)

            @classmethod
            def default(cls) -> typing.List["Constants.TaskMode.TM_Module"]:
                return []

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


class UnityConstants:
    WALL_HEIGHT = 4.0
    ATTACH_SAFE_DIST_SENSOR_TOPIC = "attach_safe_dist_sensor"

# if __name__ == "__main__":
#     rospack = rospkg.RosPack()
#     config_file_path = os.path.join(rospack.get_path('arena_bringup'), 'configs', 'task_generator.yaml')

#     rospy.init_node("task_generator_server")
#     load_parameters_from_yaml(config_file_path)
#     rospy.spin()
