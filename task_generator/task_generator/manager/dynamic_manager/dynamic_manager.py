from task_generator.shared import DynamicObstacle, Obstacle, ForbiddenZone, Model
from task_generator.simulators.base_simulator import BaseSimulator
from task_generator.manager.map_manager import MapManager
from typing import Iterable, Optional, Tuple
from geometry_msgs.msg import Point

class DynamicManager:

    namespace: str
    simulator: BaseSimulator

    def __init__(self, namespace: str, simulator: BaseSimulator):
        """
            Initialize dynamic obstacle manager.

            @namespace: global namespace
            @simulator: Simulator instance
        """
        ...

    def interactive_actor_poses_callback(self, actors):
        """
        If respawn_interactive or respawn_static are True
        the corresponding objects will be reloaded at original position
        """
        ...
    
    def dynamic_actor_poses_callback(self, actors):
        """
        If respawn_dynamic==True ,
        the given actors will be reloaded at original point with
        original waypoints
        """
        ...
        
    def spawn_obstacle(self, obstacle: Obstacle):
        """
        Loads given obstacle into the simulator. 
        If the object has an interaction radius of > 0, 
        then load it as an interactive obstacle instead of static
        """
        ...

    def spawn_dynamic_obstacle(self, obstacle: DynamicObstacle):
        """
        Loads given obstacle into the simulator.
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

    