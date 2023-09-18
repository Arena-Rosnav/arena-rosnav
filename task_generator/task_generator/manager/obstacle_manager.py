from task_generator.constants import Constants
import rospy
import numpy as np
import random

class ObstacleManager:
    def __init__(self, namespace, map_manager, simulator):
        self.map_manager = map_manager
        self.namespace = namespace
        self.simulator = simulator
        self.first_reset = True

    def start_scenario(self, scenario):
        if rospy.get_param("pedsim"):
            # self.simulator.remove_all_obstacles()
            # self.simulator.spawn_pedsim_map_borders()
            self.simulator.spawn_pedsim_map_obstacles()
            self.simulator.spawn_pedsim_dynamic_scenario_obstacles(scenario["obstacles"]["dynamic"])
            self.simulator.spawn_pedsim_static_obstacles(scenario["obstacles"]["static"])
            self.simulator.spawn_pedsim_interactive_scenario_obstacles(scenario["obstacles"]["interactive"])

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
            interactive_obstacles=Constants.ObstacleManager.INTERACTIVE_OBSTACLES,
            forbidden_zones=[]
        ):
        print("resetting random obstacles")
        if forbidden_zones is None:
            forbidden_zones = []


        if self.first_reset:
            self.first_reset = False
        else:  
            self.simulator.remove_all_obstacles()

        dynamic_obstacles_array = np.array([],dtype=object).reshape(0,3)
        static_obstacles_array = np.array([],dtype=object).reshape(0,2)
        interactive_obstacles_array = np.array([],dtype=object).reshape(0,2)
        obstacles = []

        if rospy.get_param("pedsim"):
            forbidden_zones = forbidden_zones + self.simulator.spawn_pedsim_map_obstacles()
        # print(forbidden_zones)

        # Create static obstacles
        # for i in range(random.randrange(1,10)):
        for i in range(5):
            # position = self.map_manager.get_random_pos_on_map(
            #     safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
            #     forbidden_zones=forbidden_zones,
            # )
            if rospy.get_param("pedsim"):
                x = self.simulator.create_pedsim_static_obstacle(i,self.map_manager, forbidden_zones)
                # print(x)
                forbidden_zones.append([x[1][0], x[1][1], 40])
                # print(forbidden_zones)
                static_obstacles_array = np.vstack((static_obstacles_array, x))
            else: 
                pass
                # obstacles.append(self.simulator.create_static_obstacle(position=position))

        if static_obstacles_array.size > 0:
            self.simulator.spawn_pedsim_static_obstacles(static_obstacles_array)

        # Create interactive obstacles  
        # for i in range(random.randrange(1,10)):
        for i in range(5):
            # position = self.map_manager.get_random_pos_on_map(
            #     safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
            #     forbidden_zones=forbidden_zones,
            # )
            if rospy.get_param("pedsim"):
                x = self.simulator.create_pedsim_interactive_obstacle(i,self.map_manager, forbidden_zones)
                # print(x)
                forbidden_zones.append([x[1][0], x[1][1], 40])
                # print(forbidden_zones)
                interactive_obstacles_array = np.vstack((interactive_obstacles_array, x))
            else: 
                pass
                # obstacles.append(self.simulator.create_interactive_obstacle(position=position))

        if interactive_obstacles_array.size > 0:
            self.simulator.spawn_pedsim_interactive_obstacles(interactive_obstacles_array)

        # Create dynamic obstacles 
        for i in range(random.randrange(1,10)):
            # position = self.map_manager.get_random_pos_on_map(
            #     safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
            #     forbidden_zones=forbidden_zones,
            # )
            if rospy.get_param("pedsim"):
                x = self.simulator.create_pedsim_dynamic_obstacle(i,self.map_manager, forbidden_zones)
                dynamic_obstacles_array = np.vstack((dynamic_obstacles_array, x))

            else: 
                pass
                # obstacles.append(self.simulator.create_dynamic_obstacle(position=position))

        if dynamic_obstacles_array.size > 0:
            self.simulator.spawn_pedsim_dynamic_obstacles(dynamic_obstacles_array)

    def reset_random_scenario(

            self, 
            dynamic_obstacles=Constants.ObstacleManager.DYNAMIC_OBSTACLES,
            static_obstacles=Constants.ObstacleManager.STATIC_OBSTACLES,
            interactive_obstacles=Constants.ObstacleManager.INTERACTIVE_OBSTACLES,
            forbidden_zones=[]
        ):

        print("resetting random scenario obstacles")
        if forbidden_zones is None:
            forbidden_zones = []


        if self.first_reset:
            self.first_reset = False
        else:  
            self.simulator.remove_all_obstacles()
        
        num_tables = [5,"shelf","shelf.yaml"]
        # num_chairs = [1,,""]
        num_shelves = [1,"shelf","shelf.yaml"]
        num_adults = [1,"adult","person_two_legged.model.yaml"]
        num_elder = [1,"elder","person_two_legged.model.yaml"]
        num_child = [1,"child","person_single_circle.model.yaml"]
        # num_service_robot = [1,,""]
        dynamic_obstacles_array = np.array([],dtype=object).reshape(0,3)

        obstacles = []

        if rospy.get_param("pedsim"):
            forbidden_zones = forbidden_zones + self.simulator.spawn_pedsim_map_obstacles()

        # Create static obstacles
        for ob_type in [num_tables]:
            static_obstacles_array = np.array([],dtype=object).reshape(0,2)
            for i in range(ob_type[0]):
                if rospy.get_param("pedsim"):
                    x = self.simulator.create_pedsim_static_obstacle(i,self.map_manager, forbidden_zones)
                    forbidden_zones.append([x[1][0], x[1][1], 40])
                    static_obstacles_array = np.vstack((static_obstacles_array, x))
                else: 
                    pass

            if static_obstacles_array.size > 0:
                self.simulator.spawn_pedsim_static_obstacles(static_obstacles_array, ob_type[1], ob_type[2])


        # Create interactive obstacles  
        for ob_type in [num_shelves]:
            interactive_obstacles_array = np.array([],dtype=object).reshape(0,2)
            for i in range(ob_type[0]):
                # position = self.map_manager.get_random_pos_on_map(
                #     safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
                #     forbidden_zones=forbidden_zones,
                # )
                if rospy.get_param("pedsim"):
                    x = self.simulator.create_pedsim_interactive_obstacle(i,self.map_manager, forbidden_zones)
                    # print(x)
                    forbidden_zones.append([x[1][0], x[1][1], 40])
                    # print(forbidden_zones)
                    interactive_obstacles_array = np.vstack((interactive_obstacles_array, x))
                else: 
                    pass
                    # obstacles.append(self.simulator.create_interactive_obstacle(position=position))

            if interactive_obstacles_array.size > 0:
                self.simulator.spawn_pedsim_interactive_obstacles(interactive_obstacles_array, ob_type[1], ob_type[2])

        # Create dynamic obstacles 
        for ob_type in [num_adults,num_elder,num_child]:
            for i in range(ob_type[0]):
                # position = self.map_manager.get_random_pos_on_map(
                #     safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
                #     forbidden_zones=forbidden_zones,
                # )
                if rospy.get_param("pedsim"):
                    x = self.simulator.create_pedsim_dynamic_obstacle(i,self.map_manager, forbidden_zones)
                    dynamic_obstacles_array = np.vstack((dynamic_obstacles_array, x))

                else: 
                    pass
                    # obstacles.append(self.simulator.create_dynamic_obstacle(position=position))

            if dynamic_obstacles_array.size > 0:
                self.simulator.spawn_pedsim_dynamic_obstacles(dynamic_obstacles_array, ob_type[1], ob_type[2])