from task_generator.constants import Constants
import rospy
import numpy as np
import os
import xml.etree.ElementTree as ET
import rospkg


class ObstacleManager:
    def __init__(self, namespace, map_manager, simulator):
        self.map_manager = map_manager
        self.namespace = namespace
        self.simulator = simulator
        self.first_reset = True

    def start_scenario(self, scenario):
        if rospy.get_param("pedsim"):
            self.simulator.spawn_pedsim_map_obstacles()
            self.simulator.spawn_pedsim_dynamic_scenario_obstacles(scenario["obstacles"]["dynamic"])
            self.simulator.spawn_pedsim_static_obstacles(scenario["obstacles"]["static"])
            self.simulator.spawn_pedsim_interactive_scenario_obstacles(scenario["obstacles"]["interactive"])

    def reset_scenario(self, scenario):
        if rospy.get_param("pedsim"):
            self.simulator.reset_pedsim_agents()
        self.simulator.remove_all_obstacles()

        if not scenario.get("obstacles") or not scenario.get("obstacles").get("static"):
            return

        # for obstacle in scenario["obstacles"]["static"]:
        #     self.simulator.spawn_obstacle(
        #         [*obstacle["pos"], 0],
        #         yaml_path=obstacle["yaml_path"],
        #     )

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

        # Create static obstacles
        for i in range(static_obstacles):
            if rospy.get_param("pedsim"):
                x = self.simulator.create_pedsim_static_obstacle(i,self.map_manager, forbidden_zones)
                forbidden_zones.append([x[1][0], x[1][1], 40])
                static_obstacles_array = np.vstack((static_obstacles_array, x))
            else: 
                pass
                # position = self.map_manager.get_random_pos_on_map(
                #     safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
                #     forbidden_zones=forbidden_zones,
                # )
                # obstacles.append(self.simulator.create_static_obstacle(position=position))

        if static_obstacles_array.size > 0:
            self.simulator.spawn_pedsim_static_obstacles(static_obstacles_array)

        # Create interactive obstacles  
        for i in range(interactive_obstacles):
            if rospy.get_param("pedsim"):
                x = self.simulator.create_pedsim_interactive_obstacle(i,self.map_manager, forbidden_zones)
                forbidden_zones.append([x[1][0], x[1][1], 40])
                interactive_obstacles_array = np.vstack((interactive_obstacles_array, x))
            else: 
                pass
                # position = self.map_manager.get_random_pos_on_map(
                #     safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
                #     forbidden_zones=forbidden_zones,
                # )
                # obstacles.append(self.simulator.create_interactive_obstacle(position=position))

        if interactive_obstacles_array.size > 0:
            self.simulator.spawn_pedsim_interactive_obstacles(interactive_obstacles_array)

        # Create dynamic obstacles 
        for i in range(dynamic_obstacles):
            if rospy.get_param("pedsim"):
                x = self.simulator.create_pedsim_dynamic_obstacle(i,self.map_manager, forbidden_zones)
                dynamic_obstacles_array = np.vstack((dynamic_obstacles_array, x))

            else: 
                pass
                # position = self.map_manager.get_random_pos_on_map(
                #     safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
                #     forbidden_zones=forbidden_zones,
                # )
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
        
        # print("READING XML")
        map_path = os.path.join(
            rospkg.RosPack().get_path("task_generator"), 
            "scenarios", 
            "random_scenario.xml"
        )
        tree = ET.parse(map_path)
        root = tree.getroot()
        num_tables = [int(root[0][0].text),root[0][1].text,root[0][2].text]
        num_shelves = [int(root[1][0].text),root[1][1].text,root[1][2].text]
        num_adults = [int(root[2][0].text),root[2][1].text,root[2][2].text]
        num_elder = [int(root[3][0].text),root[3][1].text,root[3][2].text]
        num_child = [int(root[4][0].text),root[4][1].text,root[4][2].text]

        dynamic_obstacles_array = np.array([],dtype=object).reshape(0,3)
        static_obstacles_array = np.array([],dtype=object).reshape(0,2)
        interactive_obstacles_array = np.array([],dtype=object).reshape(0,2)

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
                if rospy.get_param("pedsim"):
                    x = self.simulator.create_pedsim_interactive_obstacle(i,self.map_manager, forbidden_zones)
                    forbidden_zones.append([x[1][0], x[1][1], 40])
                    interactive_obstacles_array = np.vstack((interactive_obstacles_array, x))
                else: 
                    pass
                    # position = self.map_manager.get_random_pos_on_map(
                    #     safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
                    #     forbidden_zones=forbidden_zones,
                    # )
                    # obstacles.append(self.simulator.create_interactive_obstacle(position=position))

            if interactive_obstacles_array.size > 0:
                self.simulator.spawn_pedsim_interactive_obstacles(interactive_obstacles_array, ob_type[1], ob_type[2])
                # self.simulator.spawn_pedsim_interactive_obstacles(np.concatenate((interactive_obstacles_array,static_obstacles_array)), ob_type[1], ob_type[2])

        # Create dynamic obstacles 
        for ob_type in [num_adults,num_elder,num_child]:
            for i in range(ob_type[0]):
                if rospy.get_param("pedsim"):
                    x = self.simulator.create_pedsim_dynamic_obstacle(i,self.map_manager, forbidden_zones)
                    dynamic_obstacles_array = np.vstack((dynamic_obstacles_array, x))

                else: 
                    pass
                    # position = self.map_manager.get_random_pos_on_map(
                    #     safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
                    #     forbidden_zones=forbidden_zones,
                    # )
                    # obstacles.append(self.simulator.create_dynamic_obstacle(position=position))

            if dynamic_obstacles_array.size > 0:
                self.simulator.spawn_pedsim_dynamic_obstacles(dynamic_obstacles_array, ob_type[1],ob_type[2])