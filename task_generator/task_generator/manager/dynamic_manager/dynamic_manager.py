from task_generator.shared import CreatedDynamicObstacle, CreatedObstacle, ForbiddenZone, Model
from task_generator.simulators.base_simulator import BaseSimulator
from task_generator.manager.map_manager import MapManager

from typing import Iterable, Tuple

class DynamicManager:
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
        
    def spawn_obstacle(self, obstacle: CreatedObstacle, name: str, model: Model, interaction_radius: float = 0.):
        """
        Loads given obstacle into the simulator. 
        If the object has an interaction radius of > 0, 
        then load it as an interactive obstacle instead of static
        """
        ...

    def spawn_dynamic_obstacle(self, obstacle: CreatedDynamicObstacle, name: str):
        """
        Loads given obstacle into the simulator.
        Currently by loading a existing sdf file, 
        then reaplacing the static values by dynamic ones 
        """
        ...

    def spawn_map_obstacles(self):
        """
        Loads given obstacles into the simulator,
        the map file is retrieved from launch parameter "map_file"
        """
        ...

    def spawn_dynamic_scenario_obstacles(self, peds):
        """
        Loads given scenario peds into the simulator.
        To-Do: consider merging with spawn_dynamic_obstacles or simplifying by calling it
        """
        ...

    def spawn_scenario_obstacles(self, obstacles, interaction_radius):
        """
        Loads given obstacles into the simulator.
        If the object has an interaction radius of > 0, 
        then load it as an interactive obstacle instead of static
        To-Do: consider merging with spawn_obstacles or simplifying by calling it
        """
        ...

    def remove_interactive_obstacles(self):
        """
        Removes interactive obstacles from simulator.
        """
        ...

    