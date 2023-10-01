from task_generator.constants import Constants
import rospy
import numpy as np
import os
import xml.etree.ElementTree as ET
import rospkg
from task_generator.manager.dynamic_manager.dynamic_manager import DynamicManager
import time


#GRADUALLY MIGRATE ALL METHODS FOR STATIC OBSTACLES FROM DYNAMIC_MANAGERS TO HERE

class ObstacleManager:
    def __init__(self, namespace, map_manager, simulator, dynamic_manager: DynamicManager):
        self.map_manager = map_manager
        self.namespace = namespace
        self.simulator = simulator

        self.first_reset = True
        self.dynamic_manager = dynamic_manager

        self.simulator.interactive_actor_poses_callback = self.dynamic_manager.interactive_actor_poses_callback
        self.simulator.dynamic_actor_poses_callback = self.dynamic_manager.dynamic_actor_poses_callback

    # TASK MODE SCENARIO 
    def start_scenario(self, scenario):
        print("spawning scenario obstacles")

        self.dynamic_manager.spawn_map_obstacles()
        self.dynamic_manager.spawn_dynamic_scenario_obstacles(scenario["obstacles"]["dynamic"])
        self.dynamic_manager.spawn_scenario_obstacles(scenario["obstacles"]["static"], interaction_radius=0.0)
        # Most scenarion files currently without interactive objects
        # self.dynamic_manager.spawn_scenario_obstacles(scenario["obstacles"]["interactive"], interaction_radius=1.0)

    def reset_scenario(self, scenario):
        self.dynamic_manager.remove_interactive_obstacles()
        self.simulator.remove_all_obstacles()

        self.dynamic_manager.spawn_dynamic_scenario_obstacles(scenario["obstacles"]["dynamic"])
        self.dynamic_manager.spawn_scenario_obstacles(scenario["obstacles"]["static"], interaction_radius=0.0)
        # self.dynamic_manager.spawn_scenario_obstacles(scenario["obstacles"]["interactive"], interaction_radius=1.0)

    # TASK MODE RANDOM
    def reset_random(
            self, 
            dynamic_obstacles=Constants.ObstacleManager.DYNAMIC_OBSTACLES,
            static_obstacles=Constants.ObstacleManager.STATIC_OBSTACLES,
            interactive_obstacles=Constants.ObstacleManager.INTERACTIVE_OBSTACLES,
            forbidden_zones=[]
        ):

        if forbidden_zones is None:
            forbidden_zones = []

        if self.first_reset:
            self.first_reset = False
        else:  
            self.simulator.remove_all_obstacles() 
            self.dynamic_manager.remove_interactive_obstacles()

        dynamic_obstacles_array = np.array([],dtype=object).reshape(0,3)
        static_obstacles_array = np.array([],dtype=object).reshape(0,2)
        interactive_obstacles_array = np.array([],dtype=object).reshape(0,2)

        forbidden_zones = forbidden_zones + self.dynamic_manager.spawn_map_obstacles()

        # Create static obstacles
        for i in range(static_obstacles):
            x = self.dynamic_manager.create_obstacle(False, i ,self.map_manager, forbidden_zones)
            forbidden_zones.append([x[1][0], x[1][1], 40])
            static_obstacles_array = np.vstack((static_obstacles_array, x))

        if static_obstacles_array.size > 0:
            self.dynamic_manager.spawn_obstacles(static_obstacles_array, interaction_radius=0.0)

        # Create interactive obstacles  
        for i in range(interactive_obstacles):
            x = self.dynamic_manager.create_obstacle(False, i,self.map_manager, forbidden_zones)
            forbidden_zones.append([x[1][0], x[1][1], 40])
            interactive_obstacles_array = np.vstack((interactive_obstacles_array, x))

        if interactive_obstacles_array.size > 0:
            self.dynamic_manager.spawn_obstacles(interactive_obstacles_array, interaction_radius=1.0)

        # Create dynamic obstacles 
        for i in range(dynamic_obstacles):
            x = self.dynamic_manager.create_obstacle(True, i,self.map_manager, forbidden_zones)
            dynamic_obstacles_array = np.vstack((dynamic_obstacles_array, x))

        if dynamic_obstacles_array.size > 0:
            self.dynamic_manager.spawn_dynamic_obstacles(dynamic_obstacles_array)

    # TASK MODE RANDOM SCENARIO
    def reset_random_scenario(
            self, 
            dynamic_obstacles=Constants.ObstacleManager.DYNAMIC_OBSTACLES,
            static_obstacles=Constants.ObstacleManager.STATIC_OBSTACLES,
            interactive_obstacles=Constants.ObstacleManager.INTERACTIVE_OBSTACLES,
            forbidden_zones=None
        ):

        if forbidden_zones is None:
            forbidden_zones = []

        if self.first_reset:
            self.first_reset = False
        else:  
            self.simulator.remove_all_obstacles()
            self.dynamic_manager.remove_interactive_obstacles()
        
        xml_path = os.path.join(
        rospkg.RosPack().get_path("task_generator"), 
        "scenarios", 
        "random_scenario.xml")
            
        tree = ET.parse(xml_path)
        root = tree.getroot()
        num_tables = [int(root[0][0].text),root[0][1].text,root[0][2].text]
        num_shelves = [int(root[1][0].text),root[1][1].text,root[1][2].text]
        num_adults = [int(root[2][0].text),root[2][1].text,root[2][2].text]
        num_elder = [int(root[3][0].text),root[3][1].text,root[3][2].text]
        num_child = [int(root[4][0].text),root[4][1].text,root[4][2].text]

        dynamic_obstacles_array = np.array([],dtype=object).reshape(0,3)
        static_obstacles_array = np.array([],dtype=object).reshape(0,2)
        interactive_obstacles_array = np.array([],dtype=object).reshape(0,2)

        forbidden_zones = forbidden_zones + self.dynamic_manager.spawn_map_obstacles()

        # Create static obstacles
        for ob_type in [num_tables]:
            static_obstacles_array = np.array([],dtype=object).reshape(0,2)
            for i in range(ob_type[0]):
                x = self.dynamic_manager.create_obstacle(False, i,self.map_manager, forbidden_zones)
                forbidden_zones.append([x[1][0], x[1][1], 40])
                static_obstacles_array = np.vstack((static_obstacles_array, x))

            if static_obstacles_array.size > 0:
                self.dynamic_manager.spawn_obstacles(static_obstacles_array, ob_type[1], ob_type[2], interaction_radius=0.0)

        # Create interactive obstacles  
        for ob_type in [num_shelves]:
            interactive_obstacles_array = np.array([],dtype=object).reshape(0,2)
            for i in range(ob_type[0]):
                x = self.dynamic_manager.create_obstacle(False, i,self.map_manager, forbidden_zones)
                forbidden_zones.append([x[1][0], x[1][1], 40])
                interactive_obstacles_array = np.vstack((interactive_obstacles_array, x))

            if interactive_obstacles_array.size > 0:
                self.dynamic_manager.spawn_obstacles(interactive_obstacles_array, ob_type[1], ob_type[2], interaction_radius=1.0)

        # Create dynamic obstacles 
        for ob_type in [num_adults,num_elder,num_child]:
            dynamic_obstacles_array = np.array([],dtype=object).reshape(0,3)
            for i in range(ob_type[0]):
                x = self.dynamic_manager.create_obstacle(True, i,self.map_manager, forbidden_zones)
                dynamic_obstacles_array = np.vstack((dynamic_obstacles_array, x))

            if dynamic_obstacles_array.size > 0:
                self.dynamic_manager.spawn_dynamic_obstacles(dynamic_obstacles_array, ob_type[1],ob_type[2])