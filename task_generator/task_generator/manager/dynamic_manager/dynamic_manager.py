from task_generator.simulators.base_simulator import BaseSimulator

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

    def create_obstacle(self, dynamic, i, map_manager, forbidden_zones):
        """ 
        Creates and returns a newly generated obstacle Object as an array with entries: 
        0: id that can be used as a name
        1: Starting Position 3D
        2: (optional) Waypoints for obstacle
        """
        ...

    def spawn_obstacles(self, obstacles, type, yaml, interaction_radius):
        """
        Loads given obstacles into the simulator. 
        If the object has an interaction radius of > 0, 
        then load it as an interactive obstacle instead of static
        """
        ...

    def spawn_dynamic_obstacles(self, peds, type, yaml):
        """
        Loads given peds into the simulator.
        Currently by loading a existing sdf file, 
        then reaplacing the static values by dynamic ones 
        """
        ...

    def spawn_map_obstacles(self):
        """
        Loads given obstacles into the simulator,
        the map file is retireved from launch parameter "map_file"
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

    