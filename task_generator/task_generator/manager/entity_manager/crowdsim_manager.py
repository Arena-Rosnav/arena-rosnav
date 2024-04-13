import dataclasses
import functools
import json
import math
import time

import rospy


import pedsim_msgs.msg as pedsim_msgs
import geometry_msgs.msg as geometry_msgs
import pedsim_srvs.srv as pedsim_srvs
import std_srvs.srv as std_srvs

import functools

from task_generator.constants import Config, Constants, Pedsim
from task_generator.manager.entity_manager.entity_manager import EntityManager
from task_generator.manager.entity_manager.utils import (
    KnownObstacles,
    ObstacleLayer,
    SDFUtil,
    YAMLUtil,
    walls_to_obstacle,
)
from task_generator.shared import (
    DynamicObstacle,
    Model,
    ModelType,
    Namespace,
    Obstacle,
    PositionOrientation,
    Robot,
)
from task_generator.simulators.flatland_simulator import FlatlandSimulator

from typing import Callable, List

from task_generator.simulators.gazebo_simulator import GazeboSimulator
from task_generator.utils import Utils, rosparam_get

from tf.transformations import quaternion_from_euler, euler_from_quaternion

# TODO retrieve this from pedsim registry
def _get_ped_type() -> str:
    return Config.General.RNG.choice(
        ["human/adult", "human/elder"],
        p=[0.8, 0.2]
    )

# TODO structure these together
def process_SDF(name: str, base_model: Model) -> Model:
    base_desc = SDFUtil.parse(sdf=base_model.description)
    SDFUtil.set_name(sdf=base_desc, name=name, tag="actor")
    SDFUtil.delete_all(sdf=base_desc, selector=SDFUtil.SFM_PLUGIN_SELECTOR)

    # actor = SDFUtil.get_model_root(base_desc, "actor")

    # assert actor is not None, "# TODO"

    # script = actor.find(r"script")
    # if script is None:
    #     script = ET.SubElement(actor, "script")
    # script.clear()

    # # script_autostart = script.find(r"auto_start")
    # # if script_autostart is None:
    # #     script_autostart = ET.SubElement(script, "auto_start")
    # # script_autostart.text = "false"

    # trajectory = script.find(r"trajectory")
    # if trajectory is None:
    #     trajectory = ET.SubElement(script, "trajectory")
    # trajectory.clear()
    # trajectory.set("id", trajectory.get("id", "0"))
    # trajectory.set("type", trajectory.get("type", "walking"))
    # trajectory.append(ET.fromstring(
    #     "<waypoint><time>0</time><pose>0 0 0 0 0 0</pose></waypoint>"))
    # # trajectory.append(ET.fromstring("<waypoint><time>1</time><pose>0 0 0 0 0 0</pose></waypoint>"))

    pedsim_plugin = base_desc.find(f".//{SDFUtil.PEDSIM_PLUGIN_SELECTOR}")
    if pedsim_plugin is not None:
        pedsim_plugin.set("name", name)

    new_desc = SDFUtil.serialize(base_desc)
    new_model = base_model.replace(description=new_desc)

    return new_model


class CrowdsimManager(EntityManager):
    _spawn_peds_srv: rospy.ServiceProxy
    _remove_peds_srv: rospy.ServiceProxy
    _reset_peds_srv: rospy.ServiceProxy
    _respawn_obstacles_srv: rospy.ServiceProxy
    _remove_obstacles_srv: rospy.ServiceProxy
    _spawn_obstacles_srv: rospy.ServiceProxy
    _respawn_peds_srv: rospy.ServiceProxy
    _add_walls_srv: rospy.ServiceProxy
    _register_robot_srv: rospy.ServiceProxy
    _pause_simulation_srv: rospy.ServiceProxy
    _unpause_simulation_srv: rospy.ServiceProxy

    _known_obstacles: KnownObstacles

    SERVICE_RESET = ""

    SERVICE_SPAWN_PEDS = "pedsim_simulator/spawn_peds"
    SERVICE_MOVE_PEDS = "pedsim_simulator/move_peds"
    SERVICE_RESPAWN_PEDS = "pedsim_simulator/respawn_peds"
    SERVICE_RESET_ALL_PEDS = "pedsim_simulator/reset_all_peds"
    SERVICE_REMOVE_ALL_PEDS = "pedsim_simulator/remove_all_peds"

    SERVICE_ADD_WALLS = "pedsim_simulator/add_walls"
    SERVICE_CLEAR_WALLS = "pedsim_simulator/clear_walls"

    SERVICE_SPAWN_OBSTACLES = "pedsim_simulator/spawn_obstacles"
    SERVICE_RESPAWN_OBSTACLES = "pedsim_simulator/respawn_obstacles"
    SERVICE_REMOVE_ALL_OBSTACLES = "pedsim_simulator/remove_all_obstacles"

    SERVICE_REGISTER_ROBOT = "pedsim_simulator/register_robot"

    SERVICE_PAUSE_SIMULATION = "pedsim_simulator/pause_simulation"
    SERVICE_UNPAUSE_SIMULATION = "pedsim_simulator/unpause_simulation"

    TOPIC_SIMULATED_OBSTACLES = "pedsim_simulator/simulated_obstacles"
    TOPIC_SIMULATED_PEDS = "pedsim_simulator/simulated_agents"

    PARAM_NEEDS_RESPAWN_WALLS = "needs_respawn_walls"
    PARAM_NEEDS_RESPAWN_OBSTACLES = "needs_respawn_obstacles"
    PARAM_NEEDS_RESPAWN_PEDS = "needs_respawn_peds"

    _is_paused: bool
    _semaphore_reset: bool

    WALLS_ENTITY = "__WALLS"

    @staticmethod
    def convert_pose(pose: geometry_msgs.Pose) -> PositionOrientation:
        # flatland has a different coordinate system
        return PositionOrientation(
            pose.position.x,
            pose.position.y,
            -math.pi / 2
            + euler_from_quaternion(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
            )[2],
        )

    @staticmethod
    def pos_to_pose(pos: PositionOrientation) -> geometry_msgs.Pose:
        return geometry_msgs.Pose(
            position=geometry_msgs.Point(x=pos.x, y=pos.y, z=0),
            orientation=geometry_msgs.Quaternion(
                *quaternion_from_euler(0.0, 0.0, pos.orientation, axes="sxyz")
            ),
        )

    def __init__(self, namespace, simulator):
        EntityManager.__init__(self, namespace=namespace, simulator=simulator)

        self._known_obstacles = KnownObstacles()

        rospy.wait_for_service(self._namespace(self.SERVICE_SPAWN_PEDS), timeout=Config.General.WAIT_FOR_SERVICE_TIMEOUT)
        rospy.wait_for_service(self._namespace(self.SERVICE_RESPAWN_PEDS), timeout=Config.General.WAIT_FOR_SERVICE_TIMEOUT)
        rospy.wait_for_service(self._namespace(self.SERVICE_RESET_ALL_PEDS), timeout=Config.General.WAIT_FOR_SERVICE_TIMEOUT)
        rospy.wait_for_service(self._namespace(self.SERVICE_REMOVE_ALL_PEDS), timeout=Config.General.WAIT_FOR_SERVICE_TIMEOUT)

        rospy.wait_for_service(self._namespace(self.SERVICE_ADD_WALLS), timeout=Config.General.WAIT_FOR_SERVICE_TIMEOUT)
        rospy.wait_for_service(self._namespace(self.SERVICE_CLEAR_WALLS), timeout=Config.General.WAIT_FOR_SERVICE_TIMEOUT)

        rospy.wait_for_service(self._namespace(self.SERVICE_SPAWN_OBSTACLES), timeout=Config.General.WAIT_FOR_SERVICE_TIMEOUT)
        rospy.wait_for_service(
            self._namespace(self.SERVICE_RESPAWN_OBSTACLES), timeout=Config.General.WAIT_FOR_SERVICE_TIMEOUT
        )
        rospy.wait_for_service(
            self._namespace(self.SERVICE_REMOVE_ALL_OBSTACLES), timeout=Config.General.WAIT_FOR_SERVICE_TIMEOUT
        )

        rospy.wait_for_service(self._namespace(self.SERVICE_REGISTER_ROBOT), timeout=Config.General.WAIT_FOR_SERVICE_TIMEOUT)

        self._spawn_peds_srv = rospy.ServiceProxy(
            self._namespace(self.SERVICE_SPAWN_PEDS),
            pedsim_srvs.SpawnPeds,
            persistent=True,
        )
        self._respawn_peds_srv = rospy.ServiceProxy(
            self._namespace(self.SERVICE_RESPAWN_PEDS),
            pedsim_srvs.SpawnPeds,
            persistent=True,
        )
        self._remove_peds_srv = rospy.ServiceProxy(
            self._namespace(self.SERVICE_REMOVE_ALL_PEDS),
            std_srvs.SetBool,
            persistent=True,
        )
        self._reset_peds_srv = rospy.ServiceProxy(
            self._namespace(self.SERVICE_RESET_ALL_PEDS),
            std_srvs.Trigger,
            persistent=True,
        )

        self._spawn_obstacles_srv = rospy.ServiceProxy(
            self._namespace(self.SERVICE_SPAWN_OBSTACLES),
            pedsim_srvs.SpawnObstacles,
            persistent=True,
        )
        self._respawn_obstacles_srv = rospy.ServiceProxy(
            self._namespace(self.SERVICE_RESPAWN_OBSTACLES),
            pedsim_srvs.SpawnObstacles,
            persistent=True,
        )
        self._remove_obstacles_srv = rospy.ServiceProxy(
            self._namespace(self.SERVICE_REMOVE_ALL_OBSTACLES),
            std_srvs.Trigger,
            persistent=True,
        )

        self._add_walls_srv = rospy.ServiceProxy(
            self._namespace(self.SERVICE_ADD_WALLS),
            pedsim_srvs.SpawnWalls,
            persistent=True,
        )
        self._clear_walls_srv = rospy.ServiceProxy(
            self._namespace(self.SERVICE_CLEAR_WALLS), std_srvs.Trigger, persistent=True
        )

        self._register_robot_srv = rospy.ServiceProxy(
            self._namespace(self.SERVICE_REGISTER_ROBOT),
            pedsim_srvs.RegisterRobot,
            persistent=True,
        )

        self._pause_simulation_srv = rospy.ServiceProxy(
            self._namespace(self.SERVICE_PAUSE_SIMULATION),
            std_srvs.Empty,
            persistent=True,
        )
        self._unpause_simulation_srv = rospy.ServiceProxy(
            self._namespace(self.SERVICE_UNPAUSE_SIMULATION),
            std_srvs.Empty,
            persistent=True,
        )

        rospy.Subscriber(
            self._namespace(self.TOPIC_SIMULATED_OBSTACLES),
            pedsim_msgs.Obstacles,
            self._obstacle_callback,
        )
        rospy.Subscriber(
            self._namespace(self.TOPIC_SIMULATED_PEDS),
            pedsim_msgs.AgentStates,
            self._ped_callback,
        )
        rospy.Subscriber(
            "/pedsim_simulator/simulated_walls", pedsim_msgs.Walls, self._walls_callback
        )

        # temp
        def gen_JAIL_POS(steps: int, x: int = 1, y: int = 0):
            steps = max(steps, 1)
            while True:
                x += y == steps
                y %= steps
                yield PositionOrientation(-x, y, 0)
                y += 1

        self.JAIL_POS = gen_JAIL_POS(10)
        # end temp

        self._is_paused = False
        self._semaphore_reset = False

    def spawn_walls(self, walls, heightmap):
        if self.WALLS_ENTITY in self._known_obstacles:  # controversial
            return

        srv = pedsim_srvs.SpawnWallsRequest()
        srv.walls = []

        for wall in walls:
            srv.walls.append(
                pedsim_msgs.Wall(
                    start=geometry_msgs.Point(*wall[0], 0),
                    end=geometry_msgs.Point(*wall[1], 0),
                )
            )

        if self._add_walls_srv.call(
            srv
        ).success:  # TODO create combined heightmap from previous walls
            self._known_obstacles.create_or_get(
                name=self.WALLS_ENTITY,
                obstacle=walls_to_obstacle(heightmap),
                layer=ObstacleLayer.WORLD,
                pedsim_spawned=False,
            )
        else:
            rospy.logwarn("spawn walls failed!")

        # if rosparam_get(str, "world_file", "") == "generated_world":
        #     rospy.set_param(self._namespace(self.PARAM_NEEDS_RESPAWN_WALLS), True)
        return

    def spawn_obstacles(self, obstacles):
        srv = pedsim_srvs.SpawnObstaclesRequest()
        srv.obstacles = []

        self.agent_topic_str = ""

        for obstacle in obstacles:
            msg = pedsim_msgs.Obstacle()

            msg.name = obstacle.name

            # TODO create a global helper function for this kind of use case
            msg.pose = self.pos_to_pose(obstacle.position)

            interaction_radius: float = obstacle.extra.get("interaction_radius", 0.0)

            self.agent_topic_str += f",{obstacle.name}/0"

            msg.type = obstacle.extra.get("type", "")
            msg.interaction_radius = interaction_radius

            msg.yaml_path = obstacle.model.get(ModelType.YAML).path

            srv.obstacles.append(msg)  # type: ignore

            known = self._known_obstacles.get(obstacle.name)
            if known is not None:
                if known.obstacle.name != obstacle.name:
                    raise RuntimeError(
                        f"new model name {obstacle.name} does not match model name {known.obstacle.name} of known obstacle {obstacle.name} (did you forget to call remove_obstacles?)"
                    )

                known.layer = ObstacleLayer.INUSE

            else:
                known = self._known_obstacles.create_or_get(
                    name=msg.name,
                    obstacle=obstacle,
                    pedsim_spawned=False,
                    layer=ObstacleLayer.INUSE,
                )

        if not self._respawn_obstacles_srv.call(srv).success:
            rospy.logwarn(f"spawn static obstacle failed!")

        rospy.set_param(self._namespace("agent_topic_string"), self.agent_topic_str)
        rospy.set_param(self._namespace(self.PARAM_NEEDS_RESPAWN_OBSTACLES), True)

        return

    def spawn_dynamic_obstacles(self, obstacles):
        srv = pedsim_srvs.SpawnPedsRequest()
        srv.peds = []

        self.agent_topic_str = ""

        for obstacle in obstacles:
            msg = pedsim_msgs.Ped()

            msg.id = obstacle.name

            msg.pos = geometry_msgs.Point(*obstacle.position)

            self.agent_topic_str += f",{obstacle.name}/0"
            msg.type = obstacle.extra.get("type")
            msg.yaml_file = obstacle.model.get(ModelType.YAML).path

            msg.type = _get_ped_type()
            msg.number_of_peds = 1
            msg.vmax = Pedsim.VMAX(obstacle.extra.get("vmax", None))

            # TODO
            msg.configuration = json.dumps({})

            msg.waypoints = [
                geometry_msgs.Point(*waypoint) for waypoint in obstacle.waypoints
            ]

            srv.peds.append(msg)  # type: ignore

            obstacle = dataclasses.replace(
                obstacle,
                model=obstacle.model.override(
                    model_type=ModelType.SDF,
                    override=functools.partial(process_SDF, str(msg.id)),
                    name=msg.id,
                ).override(
                    model_type=ModelType.YAML,
                    override=lambda model: model.replace(
                        description=YAMLUtil.serialize(
                            YAMLUtil.update_plugins(
                                namespace=self._simulator._namespace(str(msg.id)),
                                description=YAMLUtil.parse_yaml(model.description),
                            )
                        )
                    ),
                    name=msg.id,
                ),
            )

            known = self._known_obstacles.get(msg.id)
            if known is not None:
                if known.obstacle.name != obstacle.name:
                    raise RuntimeError(
                        f"new model name {obstacle.name} does not match model name {known.obstacle.name} of known obstacle {msg.id} (did you forget to call remove_obstacles?)"
                    )

                known.layer = ObstacleLayer.INUSE
            else:
                known = self._known_obstacles.create_or_get(
                    name=msg.id,
                    obstacle=obstacle,
                    pedsim_spawned=False,
                    layer=ObstacleLayer.UNUSED,
                )

        if not self._respawn_peds_srv.call(srv).success:
            rospy.logwarn(f"spawn dynamic obstacles failed!")

        rospy.set_param(self._namespace("agent_topic_string"), self.agent_topic_str)
        rospy.set_param(self._namespace(self.PARAM_NEEDS_RESPAWN_PEDS), True)

    def unuse_obstacles(self):
        self._is_paused = True
        self._pause_simulation_srv.call(std_srvs.EmptyRequest())

        for obstacle_id, obstacle in self._known_obstacles.items():
            if obstacle.layer == ObstacleLayer.INUSE:
                obstacle.layer = ObstacleLayer.UNUSED

    def remove_obstacles(self, purge):
        # unused not always previously called
        if not self._is_paused:
            self._is_paused = True
            self._pause_simulation_srv.call(std_srvs.EmptyRequest())

        # because this is critical region
        while self._semaphore_reset:
            time.sleep(0.1)

        actions: List[Callable] = []

        self._semaphore_reset = True

        # BEGIN CRITICAL REGION
        try:
            to_forget: List[str] = list()

            if purge >= ObstacleLayer.WORLD:
                self._clear_walls_srv.call(std_srvs.TriggerRequest())

                actions.append(
                    lambda: self._simulator.delete_entity(name=self.WALLS_ENTITY)
                )
                to_forget.append(self.WALLS_ENTITY)

            for obstacle_id, obstacle in self._known_obstacles.items():
                if purge >= obstacle.layer:
                    if isinstance(self._simulator, GazeboSimulator):
                        # TODO remove this once actors can be deleted properly
                        if isinstance(obstacle.obstacle, DynamicObstacle):

                            def anon1(obstacle_id):
                                jail = next(self.JAIL_POS)
                                self._simulator.move_entity(
                                    name=obstacle_id, position=jail
                                )

                            actions.append(functools.partial(anon1, obstacle_id))

                        else:
                            # end
                            def anon2(obstacle_id):
                                obstacle.pedsim_spawned = False
                                self._simulator.delete_entity(name=obstacle_id)

                            actions.append(functools.partial(anon2, obstacle_id))
                            to_forget.append(obstacle_id)

                    else:
                        obstacle.pedsim_spawned = False
                        to_forget.append(obstacle_id)

            for obstacle_id in to_forget:
                self._known_obstacles.forget(name=obstacle_id)

        finally:
            self._semaphore_reset = False

        # END CRITICAL REGION

        for action in actions:
            action()

        self._unpause_simulation_srv.call(std_srvs.EmptyRequest())
        self._is_paused = False

    def spawn_robot(self, robot: Robot):
        self._simulator.spawn_entity(robot)

        request = pedsim_srvs.RegisterRobotRequest()

        request.name = robot.name
        request.odom_topic = self._namespace(robot.name, "odom")

        self._register_robot_srv(request)

    def move_robot(self, name: str, position: PositionOrientation):
        self._simulator.move_entity(name=name, position=position)

    def _walls_callback(self, walls: pedsim_msgs.Walls):
        if self._is_paused:
            return

        # if not rosparam_get(
        #     bool, self._namespace(self.PARAM_NEEDS_RESPAWN_WALLS), False
        # ):
        #     return
        # rospy.set_param(self._namespace(self.PARAM_NEEDS_RESPAWN_WALLS), False)

        if not Utils.is_synthetic_map():
            return

        if Utils.get_simulator() in [Constants.Simulator.FLATLAND]:
            return

        entity = self._known_obstacles.get(self.WALLS_ENTITY)

        if entity is not None and not entity.pedsim_spawned:
            self._simulator.spawn_entity(entity=entity.obstacle)
            entity.pedsim_spawned = True

    def _obstacle_callback(self, obstacles: pedsim_msgs.Obstacles):
        if self._is_paused:
            return

        # if not rosparam_get(
        #     bool, self._namespace(self.PARAM_NEEDS_RESPAWN_OBSTACLES), False
        # ):
        #     return
        # rospy.set_param(self._namespace(self.PARAM_NEEDS_RESPAWN_OBSTACLES), False)

        if isinstance(self._simulator, FlatlandSimulator):
            return  # already taken care of by pedsim

        obstacle_states: List[pedsim_msgs.Obstacle] = obstacles.obstacles or []

        for obstacle in obstacle_states:
            obstacle_name = obstacle.name

            entity = self._known_obstacles.get(obstacle_name)

            if entity is None:
                # rospy.logwarn(
                #     f"obstacle {obstacle_name} not known by {type(self).__name__} (known: {list(self._known_obstacles.keys())})")
                return

            if entity.pedsim_spawned:
                self._simulator.move_entity(
                    position=self.convert_pose(obstacle.pose), name=obstacle_name
                )

            else:
                rospy.logdebug("Spawning obstacle: name = %s", obstacle_name)

                self._simulator.spawn_entity(
                    Obstacle(
                        name=obstacle_name,
                        position=self.convert_pose(obstacle.pose),
                        model=entity.obstacle.model,
                        extra=entity.obstacle.extra,
                    )
                )

                entity.pedsim_spawned = True

        rospy.set_param(self._namespace(self.PARAM_NEEDS_RESPAWN_OBSTACLES), False)

    def _ped_callback(self, actors: pedsim_msgs.AgentStates):
        if self._is_paused:
            return

        # if not rosparam_get(
        #     bool, self._namespace(self.PARAM_NEEDS_RESPAWN_PEDS), False
        # ):
        #     return
        # rospy.set_param(self._namespace(self.PARAM_NEEDS_RESPAWN_PEDS), False)

        if isinstance(self._simulator, FlatlandSimulator):
            return  # already taken care of by pedsim

        agent_states: List[pedsim_msgs.AgentState] = actors.agent_states or []

        for actor in agent_states:
            actor_id = str(actor.id)

            entity = self._known_obstacles.get(actor_id)

            if entity is None:
                rospy.logwarn(
                    f"dynamic obstacle {actor_id} not known by {type(self).__name__}"
                )
                continue

            if entity.pedsim_spawned:
                pass  # handled by pedsim
                # self._simulator.move_entity(
                #     name=actor_id,
                #     position=(
                #         actor_pose.position.x,
                #         actor_pose.position.y,
                #         actor_pose.orientation.z + math.pi/2
                #     )
                # )

            else:
                rospy.loginfo("Spawning dynamic obstacle: actor_id = %s", actor_id)

                self._simulator.spawn_entity(
                    entity=Obstacle(
                        name=actor_id,
                        position=self.convert_pose(actor.pose),
                        model=entity.obstacle.model,
                        extra=entity.obstacle.extra,
                    )
                )

                entity.pedsim_spawned = True
