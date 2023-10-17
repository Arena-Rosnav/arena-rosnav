from io import StringIO
from typing import Any, Dict, List, Union, overload

import rospy
import os
import yaml
from flatland_msgs.srv import (
    MoveModelRequest,
    MoveModel,
    SpawnModel,
    SpawnModels,
    DeleteModel,
    DeleteModelRequest,
    DeleteModelResponse,
    DeleteModels,
    SpawnModelRequest,
    SpawnModelsRequest,
)

import flatland_msgs.msg

from task_generator.shared import ModelType

from task_generator.utils import Utils
from geometry_msgs.msg import Pose2D

from task_generator.constants import Constants
from task_generator.simulators.base_simulator import BaseSimulator
from task_generator.simulators.simulator_factory import SimulatorFactory

T = Constants.WAIT_FOR_SERVICE_TIMEOUT


@SimulatorFactory.register(Constants.Simulator.FLATLAND)
class FlatlandSimulator(BaseSimulator):
    """
    This is the flatland encoder for connecting
    flatland with the arena-benchmark task
    generator. The class implements all methods
    defined in `BaseSimulator`.

    For flatland to work properly, a dedicated .yaml
    file has to be created for each used model. This
    is, because the spawn model request only contains
    the path to this file instead of the file content
    directly. For each reset a new set of obstacles
    is created and saved in files. The path to these
    files is defined in the `tmp_model_path` param
    and defaults to `/tmp`.
    """

    _move_model_srv: rospy.ServiceProxy
    _spawn_model_srv: rospy.ServiceProxy
    _spawn_model_from_string_srv: rospy.ServiceProxy
    _spawn_models_from_string_srv: rospy.ServiceProxy
    _delete_model_srv: rospy.ServiceProxy
    _delete_models_srv: rospy.ServiceProxy

    _tmp_model_path: str

    _PLUGIN_PROPS_TO_EXTEND: Dict[str, List[str]] = {
        "DiffDrive": ["odom_pub", "twist_sub"],
        "Laser": ["topic"],
    }

    def __init__(self, namespace):
        super().__init__(namespace)
        self._namespace = namespace
        self._ns_prefix = lambda *x: os.path.join(
            "" if namespace == "" else "/" + namespace, *x)

        self._move_robot_pub = rospy.Publisher(
            self._ns_prefix("move_model"), flatland_msgs.msg.MoveModelMsg, queue_size=10
        )

        self._robot_name = rospy.get_param("robot_model", "")
        self._robot_radius = rospy.get_param("robot_radius", "")
        self._is_training_mode = rospy.get_param("train_mode", False)
        self._step_size = rospy.get_param("step_size", "")
        # self._robot_yaml_path = rospy.get_param("robot_yaml_path")
        self._tmp_model_path = str(rospy.get_param("tmp_model_path", "/tmp"))
        self._additional_full_range_laser = rospy.get_param(
            "laser/full_range_laser", False
        )

        rospy.wait_for_service(self._ns_prefix("move_model"), timeout=T)
        rospy.wait_for_service(self._ns_prefix("spawn_model"), timeout=T)
        rospy.wait_for_service(self._ns_prefix("delete_model"), timeout=T)

        self._move_model_srv = rospy.ServiceProxy(
            self._ns_prefix("move_model"), MoveModel, persistent=True
        )
        self._spawn_model_srv = rospy.ServiceProxy(
            self._ns_prefix("spawn_model"), SpawnModel
        )
        self._spawn_model[ModelType.YAML] = rospy.ServiceProxy(
            self._ns_prefix("spawn_model_from_string"), SpawnModel
        )
        self._spawn_models_from_string_srv = rospy.ServiceProxy(
            self._ns_prefix("spawn_models_from_string"), SpawnModels
        )
        self._delete_model_srv = rospy.ServiceProxy(
            self._ns_prefix("delete_model"), DeleteModel
        )
        self._delete_models_srv = rospy.ServiceProxy(
            self._ns_prefix("delete_models"), DeleteModels
        )

    def before_reset_task(self):
        pass

    def after_reset_task(self):
        pass

    def delete_obstacle(self, name):
        res: DeleteModelResponse = self._delete_model_srv(DeleteModelRequest(name=name))
        return bool(res.success)

    # SPAWN OBSTACLES
    def spawn_obstacle(self, obstacle):

        model = obstacle.model.get(self.MODEL_TYPES)

        request = SpawnModelRequest()
        request.yaml_path = model.description
        request.name = obstacle.name
        request.ns = self._namespace
        request.pose = Pose2D(
            x=obstacle.position[0], y=obstacle.position[1], theta=obstacle.position[2])

        res = self.spawn_model(model.type, request)

        return res.success

    # ROBOT
    def spawn_robot(self, robot):

        model = robot.model.get(self.MODEL_TYPES)

        file_content = self._update_plugin_topics(
            read_yaml(StringIO(model.description)), robot.name)

        request = SpawnModelRequest()
        request.yaml_path = yaml.dump(file_content)
        request.name = robot.namespace
        request.ns = robot.namespace
        request.pose = Pose2D(
            x=robot.position[0], y=robot.position[1], theta=robot.position[2])

        res = self.spawn_model(model.type, request)
        
        return res.success

    def move_entity(self, pos, name):
        pose = Pose2D()
        pose.x = pos[0]
        pose.y = pos[1]
        pose.theta = pos[2]

        move_model_request = MoveModelRequest()
        move_model_request.name = name
        move_model_request.pose = pose

        self._move_model_srv(move_model_request)

    def _update_plugin_topics(self, file_content: dict, namespace: str) -> dict:
        if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
            return file_content

        plugins = file_content["plugins"]

        for plugin in plugins:
            if self._PLUGIN_PROPS_TO_EXTEND.get(plugin["type"]):
                prop_names = self._PLUGIN_PROPS_TO_EXTEND.get(
                    plugin["type"], [])

                for name in prop_names:
                    plugin[name] = os.path.join(namespace, plugin[name])

        return file_content

    def _spawn_obstacles(self, obstacles):

        request = SpawnModelsRequest()

        models = []

        for obstacle in obstacles:
            m = flatland_msgs.msg.Model()
            m.yaml_path = obstacle[0]
            m.name = obstacle[1]
            m.ns = self._namespace
            m.pose.x = obstacle[2][0]
            m.pose.y = obstacle[2][1]
            m.pose.theta = obstacle[2][2]

            models.append(m)

        request.models = models

        self._spawn_models_from_string_srv(request)


def check_yaml_path(path: str) -> bool:
    return os.path.isfile(path)


def parse_yaml(content: str):
    return yaml.safe_load(content)


@overload
def read_yaml(yaml: StringIO): ...


@overload
def read_yaml(yaml: str): ...


def read_yaml(yaml: Union[StringIO, str]) -> Any:
    if isinstance(yaml, StringIO):
        return parse_yaml(yaml.read())

    elif isinstance(yaml, str):
        with open(yaml, "r") as file:
            return parse_yaml(file.read())

    else:
        raise ValueError(f"can't process yaml descriptor of type {type(yaml)}")
