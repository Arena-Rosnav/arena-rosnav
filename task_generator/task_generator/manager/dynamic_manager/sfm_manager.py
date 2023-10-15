# GRADUALLY REPLACE COPIED METHODS FROM THIS FILE WITH NEW NON-PEDSIM IMPLEMENTATIONS

import dataclasses
import itertools
from typing import Any, Callable, Dict, Iterable, List, Tuple
from task_generator.manager.dynamic_manager.dynamic_manager import DynamicManager
from task_generator.manager.dynamic_manager.utils import SDFUtil
from task_generator.simulators.base_simulator import BaseSimulator


from task_generator.constants import Constants

import rospy

import xml.etree.ElementTree as ET

from task_generator.shared import DynamicObstacle, Model, ModelType, ModelWrapper, PositionOrientation, Waypoint
from task_generator.utils import NamespaceIndexer

T = Constants.WAIT_FOR_SERVICE_TIMEOUT

#TODO(@voshch) make this universal
def fill_actor(model: Model, name: str, position: PositionOrientation, waypoints: Iterable[Waypoint]) -> Model:

    xml = SDFUtil.parse(model.description)

    ##### XML MODIFICATIONS

    # ACTOR
    xml_actor = SDFUtil.get_model_root(sdf=xml, tag="actor")
    assert (xml_actor is not None), "NO ACTOR FOUND IN SDF"
    xml_actor.set("name", name)

    ## ACTOR/POSE
    xml_pose = xml_actor.find("pose")
    if xml_pose is None:
        xml_pose = ET.SubElement(xml_actor, "pose")
    xml_pose.text = f"{position[0]} {position[1]} 0 0 0 {position[2]}"

    ## ACTOR/STATIC
    xml_static = xml_actor.find("static")
    if xml_static is None:
        xml_static = ET.SubElement(xml_actor, "static")
    xml_static.text = "true"

    ## ACTOR/SFM_PLUGIN
    xml_sfm_plugin = xml_actor.find(SDFUtil.SFM_PLUGIN_SELECTOR)
    if xml_sfm_plugin is None:
        xml_sfm_plugin = ET.SubElement(xml_actor, "plugin")
        xml_sfm_plugin.set("filename", 'libPedestrianSFMPlugin.so')
    xml_sfm_plugin.set("name", f"{name}_sfm_plugin")

    ### ACTOR/SFM_PLUGIN/TRAJECTORY
    xml_trajectory = xml_sfm_plugin.find("trajectory")
    if xml_trajectory is None:
        xml_trajectory = ET.fromstring("<trajectory><cyclic>true</cyclic></trajectory>")
        xml_sfm_plugin.append(xml_trajectory)
    for x,y,theta in itertools.chain([position], waypoints):
        xml_trajectory.append(ET.fromstring(f"<waypoint>{x} {y} {theta}</waypoint>"))

    # ACTOR/COLLISIONS_PLUGIN
    xml_collisions_plugin = xml_actor.find(SDFUtil.COLLISONS_PLUGIN_SELECTOR)
    if xml_collisions_plugin is None:
        xml_collisions_plugin = ET.SubElement(xml_actor, "plugin")
        xml_collisions_plugin.set("filename", 'libActorCollisionsPlugin.so')

    xml_collisions_plugin.set("name", f"{name}_collisions_plugin")
    
    #####

    return model.replace(description=SDFUtil.serialize(xml))


class SFMManager(DynamicManager):

    _spawned_obstacles: List[Tuple[str, Callable[[], Any]]]
    _namespaces: Dict[str, NamespaceIndexer]

    def __init__(self, namespace: str, simulator: BaseSimulator):
        super().__init__(namespace, simulator)

    def spawn_obstacles(self, obstacles):

        for obstacle in obstacles:

            name, free = next(self._index_namespace(obstacle.name))

            rospy.logdebug("Spawning obstacle: actor_id = %s", name)

            obstacle = dataclasses.replace(obstacle, name=name)

            name = self._simulator.spawn_obstacle(obstacle)
            self._spawned_obstacles.append((name, free))

    def spawn_dynamic_obstacles(self, obstacles):

        for obstacle in obstacles:

            name, free = next(self._index_namespace(obstacle.name))

            rospy.logdebug("Spawning dynamic obstacle: actor_id = %s", name)

            model = obstacle.model.override(
                model_type=ModelType.SDF,
                override=lambda m: fill_actor(model=m, name=name, position=obstacle.position, waypoints=obstacle.waypoints)
            )
            
            obstacle = dataclasses.replace(obstacle, name=name, model=model)

            name = self._simulator.spawn_obstacle(obstacle)
            self._spawned_obstacles.append((name, lambda:None))

    def spawn_line_obstacle(self, name, _from, _to):
        # TODO
        pass

    def remove_obstacles(self):
        for name, cleanup in self._spawned_obstacles:
            rospy.logdebug(f"Removing obstacle {name}")
            self._simulator.delete_obstacle(obstacle_id=name)
            cleanup()

        self._spawned_obstacles = []

    def _index_namespace(self, namespace: str) -> NamespaceIndexer:
        if namespace not in self._namespaces:
            self._namespaces[namespace] = NamespaceIndexer(namespace)

        return self._namespaces[namespace]
