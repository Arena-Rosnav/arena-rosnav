from task_generator.constants import Constants
import rospy
import numpy as np

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

        dynamic_obstacles_array = np.array([],dtype=object).reshape(0,3)
        static_obstacles_array = np.array([],dtype=object).reshape(0,3)
        obstacles = []
        # print('om,safe44')
        for i in range(dynamic_obstacles):
            position = self.map_manager.get_random_pos_on_map(
                safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
                forbidden_zones=forbidden_zones,
            )
            if rospy.get_param("pedsim"):
                x = self.simulator.create_pedsim_dynamic_obstacle(i,self.map_manager, forbidden_zones)
                # print(x)
                # dynamic_obstacles_array = np.append(dynamic_obstacles_array, x)
                # dynamic_obstacles_array = [dynamic_obstacles_array, x]
                dynamic_obstacles_array = np.vstack((dynamic_obstacles_array, x))
                print("printing dynamic_obstacles_array")
                print(len(dynamic_obstacles_array))
                # print('om,safe53')
            else: 
                obstacles.append(self.simulator.create_dynamic_obstacle(position=position))
        # print('om,safe53')
        # for i in range(static_obstacles):
        #     position = self.map_manager.get_random_pos_on_map(
        #         safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
        #         forbidden_zones=forbidden_zones,
        #     )
        #     if rospy.get_param("pedsim"):
        #         static_obstacles.append(self.simulator.create_pedsim_static_obstacle(self.map_manager, forbidden_zones))
        #     else: 
        #         obstacles.append(self.simulator.create_static_obstacle(position=position))

        # print(rospy.get_param("pedsim"))
        if rospy.get_param("pedsim"):
            # print("66spawn pedsim obstacles")
            self.simulator.spawn_pedsim_dynamic_obstacles(dynamic_obstacles_array)
        else: 
            self.simulator.spawn_obstacles(obstacles)