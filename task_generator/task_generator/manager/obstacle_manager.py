from task_generator.constants import Constants


class ObstacleManager:
    def __init__(self, namespace, map_manager, environment):
        self.map_manager = map_manager
        self.namespace = namespace
        self.environment = environment

    def start_scenario(self, scenario):
        self.environment.spawn_pedsim_agents(scenario["obstacles"]["dynamic"])

    def reset_scenario(self, scenario):
        self.environment.reset_pedsim_agents()

        self.environment.remove_all_obstacles()

        for obstacle in scenario["obstacles"]["static"]:
            self.environment.spawn_obstacle(
                [*obstacle["pos"], 0],
                yaml_path=obstacle["yaml_path"],
            )

    def reset_random(
            self, 
            dynamic_obstacles=Constants.ObstacleManager.DYNAMIC_OBSTACLES,
            static_obstacles=Constants.ObstacleManager.STATIC_OBSTACLES,
            forbidden_zones=[]
        ):
        self.environment.remove_all_obstacles()

        for _ in range(dynamic_obstacles):
            position = self.map_manager.get_random_pos_on_map(
                safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS, 
                forbidden_zones=forbidden_zones
            )

            self.environment.spawn_random_dynamic_obstacle(position=position)
        
        for _ in range(static_obstacles):
            position = self.map_manager.get_random_pos_on_map(
                safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS, 
                forbidden_zones=forbidden_zones
            )

            self.environment.spawn_random_static_obstacle(position=position)