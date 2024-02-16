import time
import rospy
import flatland_msgs.srv as flatland_srvs
import std_srvs.srv as std_srvs

import flatland_msgs.msg as flatland_msgs
import geometry_msgs.msg as geometry_msgs

from task_generator.shared import ModelType, Namespace

from task_generator.utils import rosparam_get

from task_generator.constants import Constants, Config
from task_generator.simulators.base_simulator import BaseSimulator
from task_generator.simulators.simulator_factory import SimulatorFactory

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

    _resume_srv: rospy.ServiceProxy
    _pause_srv: rospy.ServiceProxy

    _tmp_model_path: str
    _synchronous: bool

    def __init__(self, namespace):
        super().__init__(namespace=namespace)

        self._move_robot_pub = rospy.Publisher(
            self._namespace("move_model"), flatland_msgs.MoveModelMsg, queue_size=10
        )

        self._tmp_model_path = str(rosparam_get(str, "tmp_model_path", "/tmp"))

        rospy.wait_for_service(self._namespace("move_model"), timeout=Config.General.WAIT_FOR_SERVICE_TIMEOUT)
        rospy.wait_for_service(self._namespace("spawn_model"), timeout=Config.General.WAIT_FOR_SERVICE_TIMEOUT)
        rospy.wait_for_service(self._namespace("delete_model"), timeout=Config.General.WAIT_FOR_SERVICE_TIMEOUT)

        self._move_model_srv = rospy.ServiceProxy(
            self._namespace("move_model"), flatland_srvs.MoveModel, persistent=True
        )
        self._spawn_model_srv = rospy.ServiceProxy(
            self._namespace(
                "spawn_model_from_string"), flatland_srvs.SpawnModel
        )
        self._spawn_model[ModelType.YAML] = rospy.ServiceProxy(
            self._namespace(
                "spawn_model_from_string"), flatland_srvs.SpawnModel
        )
        self._spawn_models_from_string_srv = rospy.ServiceProxy(
            self._namespace(
                "spawn_models_from_string"), flatland_srvs.SpawnModels
        )
        self._delete_model_srv = rospy.ServiceProxy(
            self._namespace("delete_model"), flatland_srvs.DeleteModel
        )
        self._delete_models_srv = rospy.ServiceProxy(
            self._namespace("delete_models"), flatland_srvs.DeleteModels
        )

        self._pause_srv = rospy.ServiceProxy(
            self._namespace("pause"), std_srvs.Empty
        )

        self._resume_srv = rospy.ServiceProxy(
            self._namespace("resume"), std_srvs.Empty
        )

        self._synchronous = rosparam_get(
            bool, self._namespace("synchronous"), False)

    def before_reset_task(self):
        self._pause()

    def after_reset_task(self):
        self._resume()

    def delete_entity(self, name):
        req = flatland_srvs.DeleteModelRequest()
        req.name = name

        res: flatland_srvs.DeleteModelResponse = self._delete_model_srv(req)
        return bool(res.success)

    def delete_all_entities(self, names: list):
        request = flatland_srvs.DeleteModelsRequest()
        request.name = names

        res = self._delete_models_srv(request)

        return bool(res.success)

    def spawn_entity(self, entity):
        model = entity.model.get(self.MODEL_TYPES)

        request = flatland_srvs.SpawnModelRequest()
        request.yaml_path = model.description

        request.name = entity.name
        request.ns = Namespace(entity.name)
        request.pose = geometry_msgs.Pose2D(
            x=entity.position.x, y=entity.position.y, theta=entity.position.orientation
        )

        res = self.spawn_model(model.type, request)

        return res.success

    def move_entity(self, name, position):
        pose = geometry_msgs.Pose2D()
        pose.x = position.x
        pose.y = position.y
        pose.theta = position.orientation

        move_model_request = flatland_srvs.MoveModelRequest()
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

    SERVICE_CALL_TRIES = 5

    def _pause(self):
        if not self._synchronous: return
        for _ in range(self.SERVICE_CALL_TRIES):
            try: self._pause_srv()
            except rospy.ServiceException: time.sleep(0.1)
            else: return
        else: rospy.logerr(f"failed to pause in {self.SERVICE_CALL_TRIES} tries")

    def _resume(self):
        if not self._synchronous: return
        for _ in range(self.SERVICE_CALL_TRIES):
            try: self._resume_srv()
            except rospy.ServiceException: time.sleep(0.1)
            else: return
        else: rospy.logerr(f"failed to resume in {self.SERVICE_CALL_TRIES} tries")
