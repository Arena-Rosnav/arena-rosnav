# GRADUALLY REPLACE COPIED METHODS FROM THIS FILE WITH NEW NON-PEDSIM IMPLEMENTATIONS

from dataclasses import asdict
import dataclasses
from typing import Any, Callable, Dict, Iterable, List, Tuple
from task_generator.manager.dynamic_manager.dynamic_manager import DynamicManager
from task_generator.simulators.base_simulator import BaseSimulator


from task_generator.constants import Constants

import rospy


import io


import xml.etree.ElementTree as ET

from task_generator.shared import DynamicObstacle, Model, ModelType, ModelWrapper, PositionOrientation, Waypoint
from task_generator.utils import NamespaceIndexer

T = Constants.WAIT_FOR_SERVICE_TIMEOUT


def fill_actor(xml_string: str, name: str, position: PositionOrientation, waypoints: Iterable[Waypoint]) -> str:

    file = io.StringIO(xml_string)
    xml = ET.parse(file)

    xml_actor = xml.getroot()
    if xml_actor.tag != "actor":
        xml_actor = xml.find("actor")
    assert (xml_actor is not None)
    xml_actor.set("name", name)

    xml_pose = xml_actor.find("pose")
    assert (xml_pose is not None)
    xml_pose.text = f"{position[0]} {position[1]} 0 0 0 {position[2]}"

    xml_plugin = xml_actor.find(
        r"""plugin[@filename='libPedestrianSFMPlugin.so']""")
    assert (xml_plugin is not None)
    xml_plugin.append(ET.fromstring(f"<group><model>{name}</model></group>"))
    xml_plugin.set("name", f"{name}_sfm_plugin")

    file = io.StringIO()
    xml.write(file, encoding="Unicode", xml_declaration=True)
    new_xml_string = file.getvalue().replace("__waypoints__", "".join(
        [f"<waypoint>{x} {y} {theta}</waypoint>" for x, y, theta in waypoints]))

    return new_xml_string


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

            obstacle = dataclasses.replace(obstacle, name=name)

            name = self._simulator.spawn_obstacle(obstacle)
            self._spawned_obstacles.append((name, free))

    def spawn_dynamic_obstacles(self, obstacles):

        for obstacle in obstacles:

            name, free = next(self._index_namespace(obstacle.name))

            rospy.loginfo("Spawning model: actor_id = %s", name)

            model = obstacle.model.get([ModelType.SDF])

            model_desc = fill_actor(model.description, name=name, position=obstacle.position, waypoints=obstacle.waypoints)

            obstacle = dataclasses.replace(obstacle, name=name)

            obstacle.model.override(
                model_type = ModelType.SDF,
                model = Model(
                    type=model.type,
                    name=name,
                    description=model_desc,
                    path=""
                )
            )

            name = self._simulator.spawn_obstacle(obstacle)
            self._spawned_obstacles.append((name, free))

        if len(obstacles):
            rospy.set_param("respawn_dynamic", False)

    def spawn_line_obstacle(self, name, _from, _to):
        # TODO
        pass

    def remove_obstacles(self):
        for name, cleanup in self._spawned_obstacles:
            rospy.logdebug(f"removing {name}")
            self._simulator.delete_obstacle(obstacle_id=name)
            cleanup()

    def _index_namespace(self, namespace: str) -> NamespaceIndexer:
        if namespace not in self._namespaces:
            self._namespaces[namespace] = NamespaceIndexer(namespace)

        return self._namespaces[namespace]
