import rospy
from flatland_msgs.srv import (
    MoveModelRequest,
    MoveModel,
    SpawnModel,
    SpawnModels,
    DeleteModel,
    DeleteModelRequest,
    DeleteModelResponse,
    DeleteModelsRequest,
    DeleteModels,
    SpawnModelRequest,
)

import flatland_msgs.msg

from task_generator.shared import ModelType

from task_generator.utils import rosparam_get
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

    def __init__(self, namespace):
        super().__init__(namespace)

        self._move_robot_pub = rospy.Publisher(
            self._namespace("move_model"), flatland_msgs.msg.MoveModelMsg, queue_size=10
        )

        self._tmp_model_path = str(rosparam_get(str, "tmp_model_path", "/tmp"))

        rospy.wait_for_service(self._namespace("move_model"), timeout=T)
        rospy.wait_for_service(self._namespace("spawn_model"), timeout=T)
        rospy.wait_for_service(self._namespace("delete_model"), timeout=T)

        self._move_model_srv = rospy.ServiceProxy(
            self._namespace("move_model"), MoveModel, persistent=True
        )
        self._spawn_model_srv = rospy.ServiceProxy(
            self._namespace("spawn_model_from_string"), SpawnModel
        )
        self._spawn_model[ModelType.YAML] = rospy.ServiceProxy(
            self._namespace("spawn_model_from_string"), SpawnModel
        )
        self._spawn_models_from_string_srv = rospy.ServiceProxy(
            self._namespace("spawn_models_from_string"), SpawnModels
        )
        self._delete_model_srv = rospy.ServiceProxy(
            self._namespace("delete_model"), DeleteModel
        )
        self._delete_models_srv = rospy.ServiceProxy(
            self._namespace("delete_models"), DeleteModels
        )

    def before_reset_task(self):
        pass

    def after_reset_task(self):
        pass

    def delete_entity(self, name):
        req = DeleteModelRequest()
        req.name = name

        res: DeleteModelResponse = self._delete_model_srv(req)
        return bool(res.success)

    def delete_all_entities(self, names: list):
        request = DeleteModelsRequest()
        request.name = names

        res = self._delete_models_srv(request)

        return bool(res.success)

    def spawn_entity(self, entity):
        model = entity.model.get(self.MODEL_TYPES)

        request = SpawnModelRequest()
        request.yaml_path = model.description

        request.name = entity.name
        request.ns = self._namespace(entity.name)
        request.pose = Pose2D(
            x=entity.position[0], y=entity.position[1], theta=entity.position[2]
        )

        res = self.spawn_model(model.type, request)

        return res.success

    def move_entity(self, name, pos):
        pose = Pose2D()
        pose.x = pos[0]
        pose.y = pos[1]
        pose.theta = pos[2]

        move_model_request = MoveModelRequest()
        move_model_request.name = name
        move_model_request.pose = pose

        self._move_model_srv(move_model_request)

    # def _spawn_obstacles(self, obstacles):

    #     request = SpawnModelsRequest()

    #     models = []

    #     for obstacle in obstacles:
    #         m = flatland_msgs.msg.Model()
    #         m.yaml_path = obstacle[0]
    #         m.name = obstacle[1]
    #         m.ns = self._namespace
    #         m.pose.x = obstacle[2][0]
    #         m.pose.y = obstacle[2][1]
    #         m.pose.theta = obstacle[2][2]

    #         models.append(m)

    #     request.models = models

    #     self._spawn_models_from_string_srv(request)
