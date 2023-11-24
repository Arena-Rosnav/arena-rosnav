import dataclasses
import functools
import itertools
import re
from typing import Iterator, List

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from pedsim_msgs.msg import (
    AgentState,
    AgentStates,
    InteractiveObstacle,
    Ped,
    Waypoint,
    Waypoints,
)
from pedsim_srvs.srv import (
    RegisterRobot,
    RegisterRobotRequest,
    SpawnInteractiveObstacles,
    SpawnObstacle,
    SpawnPeds,
)
from std_srvs.srv import SetBool, Trigger
from task_generator.constants import Constants, Pedsim
from task_generator.manager.entity_manager.entity_manager import EntityManager
from task_generator.manager.entity_manager.utils import (
    KnownObstacles,
    SDFUtil,
    YAMLUtil,
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
from task_generator.simulators.base_simulator import BaseSimulator
from task_generator.simulators.flatland_simulator import FlatlandSimulator
from task_generator.simulators.gazebo_simulator import GazeboSimulator
from task_generator.utils import rosparam_get

T = Constants.WAIT_FOR_SERVICE_TIMEOUT

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


class PedsimManager(EntityManager):
    _spawn_peds_srv: rospy.ServiceProxy
    _remove_peds_srv: rospy.ServiceProxy
    _reset_peds_srv: rospy.ServiceProxy
    _respawn_interactive_obstacles_srv: rospy.ServiceProxy
    _remove_all_interactive_obstacles_srv: rospy.ServiceProxy
    _spawn_interactive_obstacles_srv: rospy.ServiceProxy
    _respawn_peds_srv: rospy.ServiceProxy
    _add_obstacle_srv: rospy.ServiceProxy
    _register_robot_srv: rospy.ServiceProxy

    _known_obstacles: KnownObstacles

    # TODO temporary
    _id_gen: Iterator[int]
    # end

    def __init__(self, namespace: Namespace, simulator: BaseSimulator):
        EntityManager.__init__(self, namespace=namespace, simulator=simulator)

        self._known_obstacles = KnownObstacles()

        ns = self._namespace("pedsim_simulator/spawn_peds")
        rospy.wait_for_service(
            self._namespace("pedsim_simulator/spawn_peds"), timeout=T
        )
        rospy.wait_for_service(
            self._namespace("pedsim_simulator/reset_all_peds"), timeout=T
        )
        rospy.wait_for_service(
            self._namespace("pedsim_simulator/remove_all_peds"),
            timeout=T,
        )
        rospy.wait_for_service(
            self._namespace("pedsim_simulator/respawn_peds"), timeout=T
        )
        rospy.wait_for_service(
            self._namespace("pedsim_simulator/respawn_interactive_obstacles"),
            timeout=T,
        )
        rospy.wait_for_service(
            self._namespace("pedsim_simulator/remove_all_interactive_obstacles"),
            timeout=T,
        )
        rospy.wait_for_service(
            self._namespace("pedsim_simulator/add_obstacle"), timeout=T
        )
        rospy.wait_for_service(
            self._namespace("pedsim_simulator/register_robot"), timeout=T
        )

        self._spawn_peds_srv = rospy.ServiceProxy(
            self._namespace("pedsim_simulator/spawn_peds"), SpawnPeds
        )
        self._remove_peds_srv = rospy.ServiceProxy(
            self._namespace("pedsim_simulator/remove_all_peds"), SetBool
        )
        self._reset_peds_srv = rospy.ServiceProxy(
            self._namespace("pedsim_simulator/reset_all_peds"), Trigger
        )
        self._respawn_interactive_obstacles_srv = rospy.ServiceProxy(
            self._namespace("pedsim_simulator/respawn_interactive_obstacles"),
            SpawnInteractiveObstacles,
            persistent=True,
        )

        self._remove_all_interactive_obstacles_srv = rospy.ServiceProxy(
            self._namespace("pedsim_simulator/remove_all_interactive_obstacles"),
            Trigger,
        )

        self._spawn_interactive_obstacles_srv = rospy.ServiceProxy(
            self._namespace("pedsim_simulator/spawn_interactive_obstacles"),
            SpawnInteractiveObstacles,
            persistent=True,
        )

        self._respawn_peds_srv = rospy.ServiceProxy(
            self._namespace("pedsim_simulator/respawn_peds"),
            SpawnPeds,
            persistent=True,
        )

        self._add_obstacle_srv = rospy.ServiceProxy(
            self._namespace("pedsim_simulator/add_obstacle"),
            SpawnObstacle,
            persistent=True,
        )

        self._register_robot_srv = rospy.ServiceProxy(
            self._namespace("pedsim_simulator/register_robot"),
            RegisterRobot,
            persistent=True,
        )

        rospy.set_param("respawn_dynamic", True)
        rospy.set_param("respawn_static", True)
        rospy.set_param("respawn_interactive", True)
        rospy.Subscriber(
            self._namespace("pedsim_simulator/simulated_waypoints"),
            Waypoints,
            self._interactive_actor_poses_callback,
        )
        rospy.Subscriber(
            self._namespace("pedsim_simulator/simulated_agents"),
            AgentStates,
            self._dynamic_actor_poses_callback,
        )

        # temp
        def gen_JAIL_POS(steps: int, x: int = 1, y: int = 0):
            steps = max(steps, 1)
            while True:
                x += y == steps
                y %= steps
                yield (-x, y, 0)
                y += 1

        self.JAIL_POS = gen_JAIL_POS(10)
        self._id_gen = itertools.count(20)
        # end temp

    def spawn_obstacles(self, obstacles):
        srv = SpawnInteractiveObstacles()
        srv.InteractiveObstacles = []  # type: ignore

        self.agent_topic_str = ""

        n_static_obstacles: int = 0
        n_interactive_obstacles: int = 0

        for obstacle in obstacles:
            msg = InteractiveObstacle()

            # TODO create a global helper function for this kind of use case
            msg.pose = Pose(
                position=Point(x=obstacle.position[0], y=obstacle.position[1], z=0),
                orientation=Quaternion(x=0, y=0, z=obstacle.position[2], w=1),
            )

            interaction_radius: float = obstacle.extra.get("interaction_radius", 0.0)

            if interaction_radius > 0.1:
                n_interactive_obstacles += 1
                pedsim_name = self._namespace(
                    f"interactive_obstacle_{n_interactive_obstacles}"
                )
            else:
                n_static_obstacles += 1
                pedsim_name = self._namespace(f"static_obstacle_{n_static_obstacles}")

            self.agent_topic_str += f",{pedsim_name}/0"

            msg.type = obstacle.extra.get("type", "")
            msg.interaction_radius = interaction_radius

            msg.yaml_path = obstacle.model.get(ModelType.YAML).path

            srv.InteractiveObstacles.append(msg)  # type: ignore

            known = self._known_obstacles.get(pedsim_name)
            if known is not None:
                if known.obstacle.name != obstacle.name:
                    raise RuntimeError(
                        f"new model name {obstacle.name} does not match model name {known.obstacle.name} of known obstacle {pedsim_name} (did you forget to call remove_obstacles?)"
                    )

                known.used = True

                # TODO static obstacles don't have collisions if not re-spawned but moved instead, remove this once it works without respawning
                self._simulator.delete_entity(pedsim_name)
                known.pedsim_spawned = False
                # end

            else:
                known = self._known_obstacles.create_or_get(
                    name=pedsim_name, obstacle=obstacle, pedsim_spawned=False, used=True
                )

        max_num_try = 1
        i_curr_try = 0
        rospy.logdebug("trying to call service with interactive obstacles: ")

        while i_curr_try < max_num_try:
            # try to call service
            response = self._spawn_interactive_obstacles_srv.call(
                srv.InteractiveObstacles
            )  # type: ignore

            if (
                not response.success
            ):  # if service not succeeds, do something and redo service
                # rospy.logwarn(
                #     f"spawn static obstacle failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
                i_curr_try += 1
            else:
                break
        rospy.set_param(self._namespace("agent_topic_string"), self.agent_topic_str)
        rospy.set_param("respawn_static", True)
        rospy.set_param("respawn_interactive", True)
        return

    def spawn_dynamic_obstacles(self, obstacles):
        srv = SpawnPeds()
        srv.peds = []  # type: ignore

        self.agent_topic_str = ""

        for obstacle in obstacles:
            msg = Ped()

            msg.id = next(self._id_gen)

            pedsim_name = str(msg.id)

            msg.pos = Point(*obstacle.position)

            self.agent_topic_str += f",pedsim_agent_{obstacle.name}/0"
            msg.type = obstacle.extra.get("type")
            msg.yaml_file = obstacle.model.get(ModelType.YAML).path

            msg.type = "adult"
            msg.number_of_peds = 1
            msg.vmax = Pedsim.VMAX(obstacle.extra.get("vmax", None))
            msg.start_up_mode = Pedsim.START_UP_MODE(
                obstacle.extra.get("start_up_mode", None)
            )
            msg.wait_time = Pedsim.WAIT_TIME(obstacle.extra.get("wait_time", None))
            msg.trigger_zone_radius = Pedsim.TRIGGER_ZONE_RADIUS(
                obstacle.extra.get("trigger_zone_radius", None)
            )
            msg.chatting_probability = Pedsim.CHATTING_PROBABILITY(
                obstacle.extra.get("chatting_probability", None)
            )
            msg.tell_story_probability = Pedsim.TELL_STORY_PROBABILITY(
                obstacle.extra.get("tell_story_probability", None)
            )
            msg.group_talking_probability = Pedsim.GROUP_TALKING_PROBABILITY(
                obstacle.extra.get("group_talking_probability", None)
            )
            msg.talking_and_walking_probability = (
                Pedsim.TALKING_AND_WALKING_PROBABILITY(
                    obstacle.extra.get("talking_and_walking_probability", None)
                )
            )
            msg.requesting_service_probability = Pedsim.REQUESTING_SERVICE_PROBABILITY(
                obstacle.extra.get("requesting_service_probability", None)
            )
            msg.requesting_guide_probability = Pedsim.REQUESTING_GUIDE_PROBABILITY(
                obstacle.extra.get("requesting_guide_probability", None)
            )
            msg.requesting_follower_probability = (
                Pedsim.REQUESTING_FOLLOWER_PROBABILITY(
                    obstacle.extra.get("requesting_follower_probability", None)
                )
            )
            msg.max_talking_distance = Pedsim.MAX_TALKING_DISTANCE(
                obstacle.extra.get("max_talking_distance", None)
            )
            msg.max_servicing_radius = Pedsim.MAX_SERVICING_RADIUS(
                obstacle.extra.get("max_servicing_radius", None)
            )
            msg.talking_base_time = Pedsim.TALKING_BASE_TIME(
                obstacle.extra.get("talking_base_time", None)
            )
            msg.tell_story_base_time = Pedsim.TELL_STORY_BASE_TIME(
                obstacle.extra.get("tell_story_base_time", None)
            )
            msg.group_talking_base_time = Pedsim.GROUP_TALKING_BASE_TIME(
                obstacle.extra.get("group_talking_base_time", None)
            )
            msg.talking_and_walking_base_time = Pedsim.TALKING_AND_WALKING_BASE_TIME(
                obstacle.extra.get("talking_and_walking_base_time", None)
            )
            msg.receiving_service_base_time = Pedsim.RECEIVING_SERVICE_BASE_TIME(
                obstacle.extra.get("receiving_service_base_time", None)
            )
            msg.requesting_service_base_time = Pedsim.REQUESTING_SERVICE_BASE_TIME(
                obstacle.extra.get("requesting_service_base_time", None)
            )
            msg.force_factor_desired = Pedsim.FORCE_FACTOR_DESIRED(
                obstacle.extra.get("force_factor_desired", None)
            )
            msg.force_factor_obstacle = Pedsim.FORCE_FACTOR_OBSTACLE(
                obstacle.extra.get("force_factor_obstacle", None)
            )
            msg.force_factor_social = Pedsim.FORCE_FACTOR_SOCIAL(
                obstacle.extra.get("force_factor_social", None)
            )
            msg.force_factor_robot = Pedsim.FORCE_FACTOR_ROBOT(
                obstacle.extra.get("force_factor_robot", None)
            )
            msg.waypoint_mode = Pedsim.WAYPOINT_MODE(
                obstacle.extra.get("waypoint_mode", None)
            )

            msg.waypoints = []

            for waypoint in obstacle.waypoints:
                p = Point(*waypoint)
                msg.waypoints.append(p)

            srv.peds.append(msg)  # type: ignore

            obstacle = dataclasses.replace(
                obstacle,
                model=obstacle.model.override(
                    model_type=ModelType.SDF,
                    override=functools.partial(process_SDF, str(pedsim_name)),
                    name=pedsim_name,
                ).override(
                    model_type=ModelType.YAML,
                    override=lambda model: model.replace(
                        description=YAMLUtil.serialize(
                            YAMLUtil.update_plugins(
                                namespace=self._simulator._namespace(str(pedsim_name)),
                                description=YAMLUtil.parse_yaml(model.description),
                            )
                        )
                    ),
                    name=pedsim_name,
                ),
            )

            known = self._known_obstacles.get(pedsim_name)
            if known is not None:
                # TODO temp
                if False and known.obstacle.name != obstacle.name:
                    raise RuntimeError(
                        f"new model name {obstacle.name} does not match model name {known.obstacle.name} of known obstacle {pedsim_name} (did you forget to call remove_obstacles?)"
                    )

                known.used = True
            else:
                known = self._known_obstacles.create_or_get(
                    name=pedsim_name, obstacle=obstacle, pedsim_spawned=False, used=True
                )

        max_num_try = 1
        i_curr_try = 0
        while i_curr_try < max_num_try:
            # try to call service
            response = self._respawn_peds_srv.call(srv.peds)  # type: ignore

            if (
                not response.success
            ):  # if service not succeeds, do something and redo service
                # rospy.logwarn(
                #     f"spawn human failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
                i_curr_try += 1
            else:
                break

        rospy.set_param(self._namespace("agent_topic_string"), self.agent_topic_str)
        rospy.set_param("respawn_dynamic", True)

    def spawn_line_obstacle(self, name, _from, _to):
        return

    def unuse_obstacles(self):
        self._remove_all_interactive_obstacles_srv.call()
        self._remove_peds_srv.call()
        self._id_gen = itertools.count(20)

        for obstacle in self._known_obstacles.values():
            obstacle.used = False

    def remove_obstacles(self, purge):
        to_forget: List[str] = list()

        for obstacle_id, obstacle in list(self._known_obstacles.items()):
            if purge or not obstacle.used:
                # TODO remove this once actors can be deleted properly
                if isinstance(self._simulator, GazeboSimulator) and isinstance(
                    obstacle.obstacle, DynamicObstacle
                ):
                    jail = next(self.JAIL_POS)
                    self._simulator.move_entity(name=obstacle_id, pos=jail)
                    continue
                # end

                self._simulator.delete_entity(name=obstacle_id)
                obstacle.pedsim_spawned = False
                obstacle.used = False
                to_forget.append(obstacle_id)

        for obstacle_id in to_forget:
            self._known_obstacles.forget(name=obstacle_id)

    def _interactive_actor_poses_callback(self, actors: Waypoints):
        # TODO unclean
        if isinstance(self._simulator, FlatlandSimulator):
            return

        waypoints: List[Waypoint] = actors.waypoints or []

        # only once
        if rosparam_get(bool, "respawn_interactive", False):
            for actor in filter(lambda x: "interactive" in x.name, waypoints):
                self._respawn_obstacle(actor)
            rospy.set_param("respawn_interactive", False)

        if rosparam_get(bool, "respawn_static", False):
            for actor in filter(lambda x: "static" in x.name, waypoints):
                self._respawn_obstacle(actor)
            rospy.set_param("respawn_static", False)

    def _dynamic_actor_poses_callback(self, actors: AgentStates):
        # TODO unclean
        if isinstance(self._simulator, FlatlandSimulator):
            return

        agent_states: List[AgentState] = actors.agent_states or []

        for actor in agent_states:
            actor_id = str(actor.id)

            obstacle = self._known_obstacles.get(actor_id)

            if obstacle is None:
                rospy.logwarn(
                    f"dynamic obstacle {actor_id} not known by {type(self).__name__}"
                )
                continue

            actor_pose = actor.pose

            if obstacle.pedsim_spawned:
                pass  # handled by pedsim
                # self._simulator.move_entity(
                #     name=actor_id,
                #     pos=(
                #         actor_pose.position.x,
                #         actor_pose.position.y,
                #         actor_pose.orientation.z
                #     )
                # )

            else:
                rospy.logdebug("Spawning dynamic obstacle: actor_id = %s", actor_id)

                self._simulator.spawn_entity(
                    entity=Obstacle(
                        name=actor_id,
                        position=(
                            actor_pose.position.x,
                            actor_pose.position.y,
                            actor_pose.orientation.z,
                        ),
                        model=obstacle.obstacle.model,
                        extra=obstacle.obstacle.extra,
                    )
                )

                obstacle.pedsim_spawned = True

    def _respawn_obstacle(self, actor: Waypoint):
        obstacle_name = self._namespace(str(actor.name).split("(")[0])

        obstacle = self._known_obstacles.get(obstacle_name)

        if obstacle is None:
            rospy.logwarn(
                f"obstacle {obstacle_name} not known by {type(self).__name__}"
            )
            return

        orientation = 0.0
        direction_x = 0.0
        direction_y = 0.0
        ob_type = ""

        # TODO unclean
        if not isinstance(self._simulator, FlatlandSimulator):
            orientation = float(
                re.findall(r"\(.*?\)", str(actor.name))[0]
                .replace("(", "")
                .replace(")", "")
                .replace(",", ".")
            )
            direction_x = float(
                actor.name[actor.name.index("{") + 1 : actor.name.index("}")].replace(
                    ",", "."
                )
            )
            direction_y = float(
                actor.name[actor.name.index("[") + 1 : actor.name.index("]")].replace(
                    ",", "."
                )
            )
            ob_type = actor.name[actor.name.index("&") + 1 : actor.name.index("!")]

        obstacle_position = Point(
            x=actor.position.x - direction_x,
            y=actor.position.y - direction_y,
            z=actor.position.z,
        )

        if obstacle.pedsim_spawned:
            self._simulator.move_entity(
                pos=(obstacle_position.x, obstacle_position.y, orientation),
                name=obstacle_name,
            )

        else:
            rospy.logdebug("Spawning obstacle: name = %s", obstacle_name)

            self._simulator.spawn_entity(
                Obstacle(
                    name=obstacle_name,
                    position=(obstacle_position.x, obstacle_position.y, orientation),
                    model=obstacle.obstacle.model,
                    extra=obstacle.obstacle.extra,
                )
            )

            obstacle.pedsim_spawned = True

    def spawn_robot(self, robot: Robot):
        self._simulator.spawn_entity(robot)

        request = RegisterRobotRequest()

        request.name = robot.name
        request.odom_topic = Namespace(robot.name)("odom")

        self._register_robot_srv(request)

    def move_robot(self, name: str, position: PositionOrientation):
        self._simulator.move_entity(name=name, pos=position)
