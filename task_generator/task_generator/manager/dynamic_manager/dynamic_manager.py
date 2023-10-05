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
        ...
    
    def dynamic_actor_poses_callback(self, actors):
        ...

    def create_obstacle(self, dynamic, i, map_manager, forbidden_zones):
        ...

    def spawn_obstacles(self, obstacles, type, yaml, interaction_radius):
        ...

    def spawn_dynamic_obstacles(self, peds, type, yaml):
        ...

    def spawn_map_obstacles(self):
        ...

    def spawn_dynamic_scenario_obstacles(self, peds):
        ...

    def spawn_scenario_obstacles(self, obstacles, interaction_radius):
        ...

    def remove_interactive_obstacles(self):
        ...

    