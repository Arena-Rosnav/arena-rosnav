from typing import Generator, Iterator, List
from task_generator.constants import Constants
import rospy
import numpy as np
import os
import xml.etree.ElementTree as ET
import rospkg
from task_generator.manager.dynamic_manager.dynamic_manager import DynamicManager
import time
from task_generator.manager.map_manager import MapManager

from task_generator.shared import CreatedDynamicObstacle, CreatedInteractiveObstacle, CreatedStaticObstacle, Model, ModelType

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

import itertools

from task_generator.simulators.base_simulator import BaseSimulator

#GRADUALLY MIGRATE ALL METHODS FOR STATIC OBSTACLES FROM DYNAMIC_MANAGERS TO HERE

class ObstacleManager:

    map_manager: MapManager
    namespace: str
    simulator: BaseSimulator
    dynamic_manager: DynamicManager

    first_reset: bool

    id_generator: Iterator[int]
    

    def __init__(self, namespace, map_manager, simulator, dynamic_manager: DynamicManager):
        self.map_manager = map_manager
        self.namespace = namespace
        self.simulator = simulator

        self.first_reset = True
        self.dynamic_manager = dynamic_manager

        self.simulator.interactive_actor_poses_callback = self.dynamic_manager.interactive_actor_poses_callback
        self.simulator.dynamic_actor_poses_callback = self.dynamic_manager.dynamic_actor_poses_callback

        self.id_generator = itertools.count(434)

    #
    def create_obstacle(self, dynamic: bool, forbidden_zones):
        """ 
        Creates and returns a newly generated obstacle Object as an array with entries: 
        0: id that can be used as a name
        1: Starting Position 3D
        2: (optional) Waypoints for obstacle
        """

        identifier = next(self.id_generator)

        safe_distance = 0.5
        [x, y, theta] = self.map_manager.get_random_pos_on_map(safe_distance, forbidden_zones) # check later for the need of free indicies and map papram

        if dynamic == True:
            waypoints = np.array( [x, y, 1]).reshape(1, 3) # the first waypoint
            safe_distance = 0.1 # the other waypoints don't need to avoid robot
            for j in range(10): 
                dist = 0
                while dist < 8:
                    [x2, y2, theta2] = self.map_manager.get_random_pos_on_map( safe_distance, forbidden_zones)
                    dist = np.linalg.norm([waypoints[-1,0] - x2,waypoints[-1,1] - y2])
                waypoints = np.vstack([waypoints, [x2, y2, 1]])

            ped=CreatedDynamicObstacle(id=str(identifier), position=Point(x=x, y=y, z=0.0), waypoints=waypoints)

        else:
            ped=CreatedStaticObstacle(id=str(identifier), position=Point(x=x, y=y, z=0.0))
        
        return ped
    
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
            forbidden_zones=None
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
            x = self.create_obstacle(False, forbidden_zones)
            forbidden_zones.append([x.position[0], x.position[1], 40])
            static_obstacles_array = np.vstack((static_obstacles_array, x))

        if static_obstacles_array.size > 0:
            self.dynamic_manager.spawn_obstacles(static_obstacles_array, interaction_radius=0.0)

        # Create interactive obstacles  
        for i in range(interactive_obstacles):
            x = self.create_obstacle(False, forbidden_zones)
            forbidden_zones.append([x.position[0], x.position[1], 40])
            interactive_obstacles_array = np.vstack((interactive_obstacles_array, x))

        if interactive_obstacles_array.size > 0:
            self.dynamic_manager.spawn_obstacles(interactive_obstacles_array, interaction_radius=1.0)

        # Create dynamic obstacles 
        for i in range(dynamic_obstacles):
            x = self.create_obstacle(True, forbidden_zones)
            dynamic_obstacles_array = np.vstack((dynamic_obstacles_array, x))

        if dynamic_obstacles_array.size > 0:
            self.dynamic_manager.spawn_dynamic_obstacle(dynamic_obstacles_array)

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

        dynamic_obstacles_array: List[CreatedDynamicObstacle]
        static_obstacles_array: List[CreatedStaticObstacle]
        interactive_obstacles_array: List[CreatedInteractiveObstacle]

        forbidden_zones = forbidden_zones + self.dynamic_manager.spawn_map_obstacles()

        # Create static obstacles
        for ob_type in [num_tables]:

            with open(os.path.join(rospkg.RosPack().get_path('pedsim_gazebo_plugin'), "models", f"{ob_type[1]}.sdf")) as f:
                model = Model(type=ModelType.SDF, description=f.read())

            static_obstacles_array = list()
            for i in range(ob_type[0]):
                obstacle = self.create_obstacle(False, forbidden_zones)
                forbidden_zones.append([obstacle.position.x, obstacle.position.y, 40])
                static_obstacles_array.append(obstacle)

            for obstacle in static_obstacles_array:
                self.dynamic_manager.spawn_obstacle(obstacle, ob_type[1], model=model, interaction_radius=0.0)

        # Create interactive obstacles  
        for ob_type in [num_shelves]:

            with open(os.path.join(rospkg.RosPack().get_path('pedsim_gazebo_plugin'), "models", f"{ob_type[1]}.sdf")) as f:
                model = Model(type=ModelType.SDF, description=f.read())

            interactive_obstacles_array = list()
            for i in range(ob_type[0]):
                obstacle = self.create_obstacle(False, forbidden_zones)
                forbidden_zones.append([obstacle.position.x, obstacle.position.y, 40])
                interactive_obstacles_array.append(obstacle)

            for obstacle in interactive_obstacles_array:
                self.dynamic_manager.spawn_obstacle(obstacle, name=ob_type[1], model=model, interaction_radius=1.0)

        # Create dynamic obstacles 
        for ob_type in [num_adults,num_elder,num_child]:
            dynamic_obstacles_array = list()
            for i in range(ob_type[0]):
                obstacle = self.create_obstacle(True, forbidden_zones)
                dynamic_obstacles_array.append(obstacle)

            for obstacle in dynamic_obstacles_array[:1]:
                self.dynamic_manager.spawn_dynamic_obstacle(obstacle=obstacle, name=ob_type[1])