import dataclasses
import itertools
import time
from typing import Iterable, List

from task_generator.manager.entity_manager.entity_manager import EntityManager
from task_generator.manager.entity_manager.utils import KnownObstacles, SDFUtil
from task_generator.shared import Model, ModelType, PositionOrientation, Waypoint
from task_generator.constants import Constants

import rospy

import xml.etree.ElementTree as ET


T = Constants.WAIT_FOR_SERVICE_TIMEOUT

# TODO(@voshch) make this universal


def process_SDF(model: Model, name: str, position: PositionOrientation, waypoints: Iterable[Waypoint]) -> Model:

    xml = SDFUtil.parse(model.description)

    # XML MODIFICATIONS

    # ACTOR
    xml_actor = SDFUtil.get_model_root(sdf=xml, tag="actor")
    assert (xml_actor is not None), "NO ACTOR FOUND IN SDF"
    xml_actor.set("name", name)

    # ACTOR/POSE
    xml_pose = xml_actor.find("pose")
    if xml_pose is None:
        xml_pose = ET.SubElement(xml_actor, "pose")
    xml_pose.text = f"{position[0]} {position[1]} 0 0 0 {position[2]}"

    # ACTOR/STATIC
    xml_static = xml_actor.find("static")
    if xml_static is None:
        xml_static = ET.SubElement(xml_actor, "static")
    xml_static.text = "true"

    # ACTOR/SFM_PLUGIN
    xml_sfm_plugin = xml_actor.find(SDFUtil.SFM_PLUGIN_SELECTOR)
    if xml_sfm_plugin is None:
        xml_sfm_plugin = ET.SubElement(xml_actor, "plugin")
        xml_sfm_plugin.set("filename", 'libPedestrianSFMPlugin.so')
    xml_sfm_plugin.set("name", f"{name}_sfm_plugin")

    # ACTOR/SFM_PLUGIN/TRAJECTORY
    xml_trajectory = xml_sfm_plugin.find("trajectory")
    if xml_trajectory is None:
        xml_trajectory = ET.fromstring(
            "<trajectory><cyclic>true</cyclic></trajectory>")
        xml_sfm_plugin.append(xml_trajectory)
    for x, y, theta in itertools.chain([position], waypoints):
        xml_trajectory.append(ET.fromstring(
            f"<waypoint>{x} {y} {theta}</waypoint>"))

    # ACTOR/COLLISIONS_PLUGIN
    xml_collisions_plugin = xml_actor.find(SDFUtil.COLLISONS_PLUGIN_SELECTOR)
    if xml_collisions_plugin is None:
        xml_collisions_plugin = ET.SubElement(xml_actor, "plugin")
        xml_collisions_plugin.set("filename", 'libActorCollisionsPlugin.so')

    xml_collisions_plugin.set("name", f"{name}_collisions_plugin")

    #####

    SDFUtil.delete_all(sdf=xml, selector=SDFUtil.PEDSIM_PLUGIN_SELECTOR)

    return model.replace(description=SDFUtil.serialize(xml))


class SFMManager(EntityManager):

    _known_obstacles: KnownObstacles

    def __init__(self, namespace, simulator):
        super().__init__(namespace=namespace, simulator=simulator)
        self._known_obstacles = KnownObstacles()

    def spawn_obstacles(self, obstacles):

        for obstacle in obstacles:

            rospy.logdebug("Spawning obstacle: actor_id = %s", obstacle.name)

            obstacle = dataclasses.replace(obstacle, name=obstacle.name)

            # TODO aggregate deletes & spawns and leverage simulator parallelization
            known = self._known_obstacles.get(obstacle.name)
            if known is not None:
                if known.obstacle.name != obstacle.name:
                    raise RuntimeError(f"new model name {obstacle.name} does not match model name {known.obstacle.name} of known obstacle {obstacle.name} (did you forget to call remove_obstacles?)")

                self._simulator.move_entity(obstacle.name, obstacle.position)
                known.used = True
            else:
                known = self._known_obstacles.create_or_get(
                    name=obstacle.name,
                    obstacle=obstacle
                )
                self._simulator.spawn_entity(known.obstacle)
                known.used = True

            time.sleep(0.05)

    def spawn_dynamic_obstacles(self, obstacles):

        for obstacle in obstacles:

            name = obstacle.name

            rospy.logdebug("Spawning dynamic obstacle: actor_id = %s", name)

            model = obstacle.model.override(
                model_type=ModelType.SDF,
                override=lambda m: process_SDF(
                    model=m, name=name, position=obstacle.position, waypoints=obstacle.waypoints)
            )

            obstacle = dataclasses.replace(obstacle, name=name, model=model)


            # TODO aggregate deletes & spawns and leverage simulator parallelization
            known = self._known_obstacles.get(obstacle.name)
            if known is not None:
                if known.obstacle.name != obstacle.name:
                    raise RuntimeError(f"new model name {obstacle.name} does not match model name {known.obstacle.name} of known obstacle {obstacle.name} (did you forget to call remove_obstacles?)")

                self._simulator.move_entity(obstacle.name, obstacle.position)
                known.used = True

            else:
                known = self._known_obstacles.create_or_get(
                    name=obstacle.name,
                    obstacle=obstacle,
                    used=True
                )
                self._simulator.spawn_entity(known.obstacle)
                known.used = True

            time.sleep(0.05)

    def spawn_line_obstacle(self, name, _from, _to):
        # TODO
        pass

    def unuse_obstacles(self):

        self.remove_obstacles(purge=True) # neither obstacle type can be used without respawning in gazebo (static obstacles lose collisions when moved, actor sfm plugin can't be modified on-the-fly)

        for obstacle in self._known_obstacles.values():
            obstacle.used = False

    def remove_obstacles(self, purge):
        to_forget: List[str] = list()

        for obstacle_id, obstacle in list(self._known_obstacles.items()):
            if purge or not obstacle.used:
                self._simulator.delete_entity(name=obstacle_id)
                obstacle.used = False
                time.sleep(0.05)
                to_forget.append(obstacle_id)
                

        for obstacle_id in to_forget:
            self._known_obstacles.forget(name=obstacle_id)