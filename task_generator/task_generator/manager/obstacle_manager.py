from task_generator.constants import Constants


class ObstacleManager:
    def __init__(self, namespace, map_manager, simulator):
        self.map_manager = map_manager
        self.namespace = namespace
        self.simulator = simulator

    def start_scenario(self, scenario):
        self.simulator.spawn_pedsim_agents(scenario["obstacles"]["dynamic"])

    def reset_scenario(self, scenario):
        self.simulator.reset_pedsim_agents()

        self.simulator.remove_all_obstacles()

        print(scenario.get("obstacles").get("static"))

        if not scenario.get("obstacles") or not scenario.get("obstacles").get("static"):
            return

        for obstacle in scenario["obstacles"]["static"]:
            self.simulator.spawn_obstacle(
                [*obstacle["pos"], 0],
                yaml_path=obstacle["yaml_path"],
            )

    def reset_random(
            self, 
            dynamic_obstacles=Constants.ObstacleManager.DYNAMIC_OBSTACLES,
            static_obstacles=Constants.ObstacleManager.STATIC_OBSTACLES,
            forbidden_zones=[]
        ):
        if forbidden_zones is None:
            forbidden_zones = []

        self.simulator.remove_all_obstacles()

        obstacles = []

<<<<<<< HEAD
        for _ in range(dynamic_obstacles):
=======
        # Create dynamic obstacles # TODO dynamic
        for i in range(0):
            position = self.map_manager.get_random_pos_on_map(
                safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
                forbidden_zones=forbidden_zones,
            )
            if rospy.get_param("pedsim"):
                x = self.simulator.create_pedsim_dynamic_obstacle(i,self.map_manager, forbidden_zones)
                dynamic_obstacles_array = np.vstack((dynamic_obstacles_array, x))

            else: 
                obstacles.append(self.simulator.create_dynamic_obstacle(position=position))

        # Create interactive obstacles # TODO dynamic
        for i in range(0):
>>>>>>> parent of 95a2f97... cleaning up prints and comments
            position = self.map_manager.get_random_pos_on_map(
                safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
                forbidden_zones=forbidden_zones,
            )
            obstacles.append(self.simulator.create_dynamic_obstacle(position=position))

        for _ in range(static_obstacles):
            position = self.map_manager.get_random_pos_on_map(
                safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
                forbidden_zones=forbidden_zones,
            )
            obstacles.append(self.simulator.create_static_obstacle(position=position))

<<<<<<< HEAD
        self.simulator.spawn_obstacles(obstacles)
=======
        # Spawn obstacles
        # TODO better solution instead of param
        if rospy.get_param("pedsim"):
            self.simulator.spawn_pedsim_static_obstacles(static_obstacles_array)
            # self.simulator.spawn_pedsim_interactive_obstacles(interactive_obstacles_array)
            # self.simulator.spawn_pedsim_dynamic_obstacles(dynamic_obstacles_array)
            self.simulator.spawn_pedsim_map_borders()
        else: 
            self.simulator.spawn_obstacles(obstacles)
>>>>>>> parent of 95a2f97... cleaning up prints and comments
