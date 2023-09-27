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