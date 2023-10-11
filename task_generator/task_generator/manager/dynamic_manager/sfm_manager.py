# GRADUALLY REPLACE COPIED METHODS FROM THIS FILE WITH NEW NON-PEDSIM IMPLEMENTATIONS

from dataclasses import asdict
import dataclasses
from typing import Any, Callable, Dict, Iterable, List, Tuple
from task_generator.manager.dynamic_manager.dynamic_manager import DynamicManager
from task_generator.manager.dynamic_manager.utils import SDFUtil
from task_generator.simulators.base_simulator import BaseSimulator


from task_generator.constants import Constants

import rospy


import io


import xml.etree.ElementTree as ET

from task_generator.shared import DynamicObstacle, Model, ModelType, ModelWrapper, PositionOrientation, Waypoint
from task_generator.utils import NamespaceIndexer

T = Constants.WAIT_FOR_SERVICE_TIMEOUT


def fill_actor(xml_string: str, name: str, position: PositionOrientation, waypoints: Iterable[Waypoint]) -> str:

    xml = SDFUtil.parse(xml_string)

    xml_actor = SDFUtil.get_model_root(sdf=xml, tag="actor")

    assert (xml_actor is not None)
    xml_actor.set("name", name)

    xml_pose = xml_actor.find("pose")
    assert (xml_pose is not None)
    xml_pose.text = f"{position[0]} {position[1]} 0 0 0 {position[2]}"

    xml_plugin = xml_actor.find(SDFUtil.SFM_PLUGIN_SELECTOR)

    if xml_plugin is None:
        xml_plugin = ET.SubElement(xml_actor, "plugin")
        xml_plugin.set("filename", 'libPedestrianSFMPlugin.so')

    xml_plugin.append(ET.fromstring(f"<group><model>{name}</model></group>"))
    xml_plugin.set("name", f"{name}_sfm_plugin")

    xml_trajectory = ET.fromstring("<trajectory><cyclic>true</cyclic></trajectory>")
    for x,y,theta in waypoints:
        xml_trajectory.append(ET.fromstring(f"<waypoint>{x} {y} {theta}</waypoint>"))
        
    xml_plugin.append(xml_trajectory)

    return SDFUtil.serialize(xml)


class SFMManager(DynamicManager):

    _spawned_obstacles: List[Tuple[str, Callable[[], Any]]]
    _namespaces: Dict[str, NamespaceIndexer]

    def __init__(self, namespace: str, simulator: BaseSimulator):
        super().__init__(namespace, simulator)

        rospy.set_param("respawn_dynamic", True)
        rospy.set_param("respawn_static", True)
        rospy.set_param("respawn_interactive", True)

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

            model = obstacle.model.get([ModelType.SDF])

            model_desc = fill_actor(
                model.description,
                name=name,
                position=obstacle.position,
                waypoints=obstacle.waypoints
            )

            model = obstacle.model.override(
                model_type=ModelType.SDF,
                model=Model(
                    type=model.type,
                    name=name,
                    description=model_desc,
                    path=""
                )
            )

            obstacle = dataclasses.replace(obstacle, name=name, model=model)

            name = self._simulator.spawn_obstacle(obstacle)
            self._spawned_obstacles.append((name, free))

        if len(obstacles):
            rospy.set_param("respawn_dynamic", False)

    def spawn_line_obstacle(self, name, _from, _to):
        # TODO
        pass

    def remove_obstacles(self):
        for name, cleanup in self._spawned_obstacles:
            rospy.logdebug(f"Removing obstacle {name}")
            self._simulator.delete_obstacle(obstacle_id=name)
            cleanup()

    def _index_namespace(self, namespace: str) -> NamespaceIndexer:
        if namespace not in self._namespaces:
            self._namespaces[namespace] = NamespaceIndexer(namespace)

        return self._namespaces[namespace]
