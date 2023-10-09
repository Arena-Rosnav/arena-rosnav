import os
from rospkg import RosPack
import rospy
from task_generator.shared import DynamicObstacle, ModelType, ModelWrapper, Obstacle, Model
from task_generator.simulators.base_simulator import BaseSimulator
from typing import Callable, Collection, Dict
from geometry_msgs.msg import Point

from geometry_msgs.msg import PoseStamped


class DynamicManager:

    _namespace: str
    _simulator: BaseSimulator

    _robot_name: str
    _ns_prefix: Callable[..., str]
    _goal_pub: rospy.Publisher

    # TODO temporary
    _default_actor_model: ModelWrapper

    def __init__(self, namespace: str, simulator: BaseSimulator):
        """
            Initialize dynamic obstacle manager.

            @namespace: global namespace
            @simulator: Simulator instance
        """

        self._simulator = simulator
        self._namespace = namespace

        pkg_path = RosPack().get_path('arena-simulation-setup')
        default_actor_model_name = "actor2"

        #TODO get rid of this once random_scenario loads models randomly
        actor_model_name: str = str(rospy.get_param(
            '~actor_model_name', default_actor_model_name))
        
        actor_model_path = os.path.join(pkg_path, "dynamic_obstacles", actor_model_name)

        model_dict: Dict[ModelType, Model] = dict()

        with open(os.path.join(actor_model_path, "model.sdf")) as f:
            model_dict[ModelType.SDF] = Model(type=ModelType.SDF, description=f.read(), name="actor2", path=os.path.join(actor_model_path, "model.sdf"))

        with open(os.path.join(actor_model_path, f"{actor_model_name}.model.yaml")) as f:
            model_dict[ModelType.YAML] = Model(type=ModelType.YAML, description=f.read(), name="actor2", path=os.path.join(actor_model_path, f"{actor_model_name}.model.yaml"))

        self._default_actor_model = ModelWrapper.Constant(
            name="actor2",
            models=model_dict
        )

        self._ns_prefix = lambda *topic: os.path.join(namespace, *topic)
        self._goal_pub = rospy.Publisher(self._ns_prefix(
            "/goal"), PoseStamped, queue_size=1, latch=True)

        self._robot_name = str(rospy.get_param("robot_model", ""))

        self._spawned_obstacles = []
        self._namespaces = dict()

    def spawn_obstacles(self, obstacles: Collection[Obstacle]):
        """
        Loads given obstacles into the simulator. 
        If the object has an interaction radius of > 0, 
        then load it as an interactive obstacle instead of static
        """
        ...

    def spawn_dynamic_obstacles(self, obstacles: Collection[DynamicObstacle]):
        """
        Loads given obstacles into the simulator.
        Currently by loading a existing sdf file, 
        then reaplacing the static values by dynamic ones 
        """

    def spawn_line_obstacle(self, name: str, _from: Point, _to: Point):
        """
        Creates a line obstacle.
        """
        ...

    def remove_obstacles(self):
        """
        Removes obstacles from simulator.
        """
        ...
