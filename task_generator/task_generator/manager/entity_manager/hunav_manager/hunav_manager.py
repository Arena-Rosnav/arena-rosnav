import functools
import math
import os
import time
from threading import Lock
from typing import Any, Callable, Collection, Dict, List
import typing

import rclpy
import geometry_msgs
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from hunav_msgs.msg import Agent, AgentBehavior, Agents
from hunav_msgs.srv import (ComputeAgent, ComputeAgents, MoveAgent, ResetAgents)

from task_generator.manager.entity_manager import EntityManager
from task_generator.manager.entity_manager.hunav_manager import HunavDynamicObstacle
from task_generator.manager.entity_manager.utils import (KnownObstacles,
                                                         ObstacleLayer,
                                                         walls_to_obstacle)

from task_generator.shared import (DynamicObstacle, Model,ModelWrapper, ModelType,
                                   Namespace, PositionOrientation, Robot, Obstacle)
from task_generator.simulators import BaseSimulator
from task_generator.utils.geometry import quaternion_from_euler
from task_generator.manager.world_manager.utils import WorldMap, WorldWalls
from task_generator.manager.world_manager import WorldManager
from .import SKIN_TYPES 
import traceback
import attr


class HunavManager(EntityManager):
    """HunavManager with debug logging for tracking execution flow"""
    
    # Service Names
    SERVICE_COMPUTE_AGENT = 'compute_agent'
    SERVICE_COMPUTE_AGENTS = 'compute_agents'
    SERVICE_MOVE_AGENT = 'move_agent'
    SERVICE_RESET_AGENTS = 'reset_agents'

    SKIN_TYPES = {
            0: 'elegant_man.dae',
            1: 'casual_man.dae',
            2: 'elegant_woman.dae',
            3: 'regular_man.dae',
            4: 'worker_man.dae',
            5: 'walk.dae'
        }

    def __init__(self, namespace: Namespace, simulator: BaseSimulator, world_manager: WorldManager):
        super().__init__(namespace=namespace, simulator=simulator)
        self._world_manager = world_manager   ## To get wallobstacles from worldmanager
        """Initialize HunavManager with debug logging"""
        self.__logger = self.node.get_logger().get_child('hunav_EM')
        self.node.get_logger().warn("=== HUNAVMANAGER INIT START ===")
        self.node.get_logger().warn("Parent class initialized")
    
        # Initialize state variables
        self.node.get_logger().warn("Initializing state variables...")
        self._lock = Lock()
        self._update_rate = 0.1
        self.node.get_logger().warn("State variables initialized")
        # Initialize collections
        self.node.get_logger().warn("Initializing collections...")
        self._known_obstacles = KnownObstacles()
        self._pedestrians = {}
        self.node.get_logger().warn("Collections initialized")

        # Setup services
        self.node.get_logger().warn("Setting up services...")
        setup_success = self.setup_services()
        if not setup_success:
            self.node.get_logger().error("Service setup failed!")
        else:
            self.node.get_logger().warn("Services setup complete")

        # Wait to be sure that all services are ready
        self.node.get_logger().warn("Waiting for services to be ready...")
        time.sleep(2.0)
        self.node.get_logger().warn("Service wait complete")


        # Create timer after services are ready
        self._update_timer = self.node.create_timer(
        0.1, #update rate 
        self._update_agents,
        callback_group=rclpy.callback_groups.ReentrantCallbackGroup()
    )
        self.node.get_logger().warn("Update timer created")

        # Initialize JAIL_POS generator
        self.node.get_logger().warn("Initializing JAIL_POS generator...")
        
        def gen_JAIL_POS(steps: int, x: int = 1, y: int = 0):
            steps = max(steps, 1)
            while True:
                x += y == steps
                y %= steps
                yield PositionOrientation(-x, y, 0)
                y += 1
        self.JAIL_POS = gen_JAIL_POS(10)
        self.node.get_logger().warn("JAIL_POS generator initialized")

        self.node.get_logger().warn("=== HUNAVMANAGER INIT COMPLETE ===")


    @property 
    def world(self):
        return self._world_manager.world

    def _get_wall_points(self):
        points = []
        walls = self._world_manager.detected_walls  # Nutze detected_walls statt entities.walls
        #self.node.get_logger().warn(f"Found {len(walls)} walls from detected_walls")
        
        for wall in walls:
            wall_points = self._wall_to_points(wall)
            #self.node.get_logger().warn(f"Wall: {wall.Start} -> {wall.End}: {len(wall_points)} points")
            points.extend(wall_points)
            
        return points

    def _wall_to_points(self, wall, spacing=0.5):
        points = []
        dx = wall.End.x - wall.Start.x 
        dy = wall.End.y - wall.Start.y
        length = math.sqrt(dx*dx + dy*dy)
        steps = max(1, int(length / spacing))
        
        for i in range(steps + 1):
            t = i/steps
            point = geometry_msgs.msg.Point()
            point.x = wall.Start.x + t*dx
            point.y = wall.Start.y + t*dy
            point.z = 0.0
            points.append(point)
        return points


    def _update_agents(self):
        """Updates agent positions"""
        try:
            #self.node.get_logger().warn(f"PEDESTRIANS UPDATE: {self._pedestrians}")
            if not self._pedestrians:
                return
                
            self.node.get_logger().warn(f"Starting update for {len(self._pedestrians)} pedestrians")
            
            with self._lock:
                # Erstelle Request mit aktuellen Agenten
                request = ComputeAgents.Request()
                request.robot = self._create_robot_message()
                request.current_agents = self._get_current_agents()
                
                # Sende an HuNav
                self.node.get_logger().warn("Sending request to HuNav")
                response = self._compute_agents_client.call(request)
                
                if response and response.updated_agents.agents:
                    self.node.get_logger().warn("Received updated positions from HuNav")
                    
                    # Für jeden aktualisierten Agenten
                    for agent in response.updated_agents.agents:
                        self.node.get_logger().warn(
                            f"Processing updates for {agent.name}:"
                            f"\n Old pos: ({self._pedestrians[agent.id]['agent'].position.position.x:.2f}, "
                            f"{self._pedestrians[agent.id]['agent'].position.position.y:.2f})"
                            f"\n New pos: ({agent.position.position.x:.2f}, {agent.position.position.y:.2f})"
                        )
                        
                        # Update Position in Gazebo
                        entity_name = f"agent{agent.id}"
                        if entity_name in self._simulator.entities:
                            new_position = PositionOrientation(
                                x=agent.position.position.x,
                                y=agent.position.position.y,
                                orientation=agent.yaw
                            )
                            
                            # Bewege in Simulator
                            success = self._simulator.move_entity(entity_name, new_position)
                            
                            if success:
                                # Update gespeicherten Zustand
                                self._pedestrians[agent.id]['agent'] = agent
                                self._pedestrians[agent.id]['last_update'] = self.node.get_clock().now()
                                self.node.get_logger().warn(f"Successfully updated {agent.name}")
                    
        except Exception as e:
            self.__logger.error(f"Error in agent update: {str(e)}")
            self.__logger.error(traceback.format_exc())



    def _get_current_agents(self) -> Agents:
        """Creates current agents message for service call"""
        self.node.get_logger().warn("Creating current agents message")
        current_agents = Agents()
        current_agents.header.stamp = self.node.get_clock().now().to_msg()
        current_agents.header.frame_id = "map"
        
        # Log was wir in _pedestrians haben
        self.node.get_logger().warn(f"Current pedestrians: {list(self._pedestrians.keys())}")
        
        for agent_id, agent_data in self._pedestrians.items():
            self.node.get_logger().warn(f"iteration_agent_id: {agent_id}")
            self.node.get_logger().warn(f"iteration_agent_data: {agent_data}")
            if 'agent' in agent_data:
                current_agent = agent_data['agent']
                self.node.get_logger().warn(
                    f"Adding agent {current_agent.name} at position "
                    f"({current_agent.position.position.x}, {current_agent.position.position.y})"
                )
                current_agents.agents.append(current_agent)
        
        return current_agents


    def setup_services(self):
        """Initialize all required services with debug logging"""
        self.node.get_logger().warn("=== SETUP_SERVICES START ===")
        
        # Debug namespace information
        self.node.get_logger().warn(f"Node namespace: {self.node.get_namespace()}")
        self.node.get_logger().warn(f"Task generator namespace: {self._namespace}")
        
        # Create service names with full namespace path
        service_names = {
            'compute_agent': f'/{self.SERVICE_COMPUTE_AGENT}',
            'compute_agents': f'/{self.SERVICE_COMPUTE_AGENTS}',
            'move_agent': f'/{self.SERVICE_MOVE_AGENT}',
            'reset_agents': f'/{self.SERVICE_RESET_AGENTS}'
        }
        
        # Log service creation attempts
        for service, full_name in service_names.items():
            self.node.get_logger().warn(f"Creating service client for {service} at: {full_name}")

        # Create service clients
        self.node.get_logger().warn("Creating compute_agent client...")
        self._compute_agent_client = self.node.create_client(
            ComputeAgent,
            service_names['compute_agent']
        )
        
        self.node.get_logger().warn("Creating compute_agents client...")
        self._compute_agents_client = self.node.create_client(
            ComputeAgents,
            service_names['compute_agents']
        )
        
        self.node.get_logger().warn("Creating move_agent client...")
        self._move_agent_client = self.node.create_client(
            MoveAgent,
            service_names['move_agent']
        )
        
        self.node.get_logger().warn("Creating reset_agents client...")
        self._reset_agents_client = self.node.create_client(
            ResetAgents,
            service_names['reset_agents']
        )
        

        # Wait for Services
        required_services = [
            (self._compute_agent_client, 'compute_agent'),
            (self._compute_agents_client, 'compute_agents'),
            (self._move_agent_client, 'move_agent'),
            (self._reset_agents_client, 'reset_agents')
        ]

        max_attempts = 5
        for client, name in required_services:
            attempts = 0
            self.node.get_logger().warn(f"Waiting for service {name}...")
            
            while attempts < max_attempts:
                if client.wait_for_service(timeout_sec=2.0):
                    self.node.get_logger().warn(f'Service {name} is available')
                    break
                attempts += 1
                self.node.get_logger().warn(
                    f'Waiting for service {name} (attempt {attempts}/{max_attempts})\n'
                    f'Looking for service at: {service_names[name]}'
                )

            if attempts >= max_attempts:
                self.node.get_logger().error(
                    f'Service {name} not available after {max_attempts} attempts\n'
                    f'Was looking for service at: {service_names[name]}'
                )
                self.node.get_logger().warn("=== SETUP_SERVICES FAILED ===")
                return False

        self.node.get_logger().warn("=== SETUP_SERVICES COMPLETE ===")
        return True


    def create_pedestrian_sdf(self, agent_config: HunavDynamicObstacle) -> str:
        """Create SDF description for pedestrian using gz-sim actor format"""
        # Get skin type
        skin_type = self.SKIN_TYPES.get(agent_config.skin, 'casual_man.dae')
        
        # Animation mapping based on behavior
        ANIMATION_MAP = {
            AgentBehavior.BEH_REGULAR: "07_01-walk.bvh",
            AgentBehavior.BEH_IMPASSIVE: "69_02_walk_forward.bvh", 
            AgentBehavior.BEH_SURPRISED: "137_28-normal_wait.bvh",
            AgentBehavior.BEH_SCARED: "142_17-walk_scared.bvh",
            AgentBehavior.BEH_CURIOUS: "07_04-slow_walk.bvh",
            AgentBehavior.BEH_THREATENING: "17_01-walk_with_anger.bvh"
        }
        
        animation_file = ANIMATION_MAP.get(agent_config.behavior.type, "07_01-walk.bvh")

        # Get workspace root
        def get_workspace_root():
            current_dir = os.path.abspath(__file__)
            workspace_root = current_dir
            while not workspace_root.endswith("arena4_ws"):
                workspace_root = os.path.dirname(workspace_root)
            return workspace_root

        # Construct paths    
        mesh_path = os.path.join(
            get_workspace_root(),
            'src/deps/hunav/hunav_sim/hunav_rviz2_panel/meshes/models',
            skin_type
        )
        
        animation_path = os.path.join(
            get_workspace_root(),
            'src/deps/hunav/hunav_sim/hunav_rviz2_panel/meshes/animations',
            animation_file
        )

        sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.9">
            <actor name="{agent_config.name}">
                <pose>{agent_config.position.x} {agent_config.position.y} {self._get_agent_height(agent_config.skin)} 0 0 {agent_config.yaw}</pose>
                
                <skin>
                    <filename>{mesh_path}</filename>
                    <scale>1.0</scale>
                </skin>
                
                <animation name="walking">
                    <filename>{animation_path}</filename>
                    <scale>1.0</scale>
                    <interpolate_x>true</interpolate_x>
                </animation>

                <plugin name="HuNavActorPluginIGN" filename="libHuNavActorPluginIGN.so">
                    <update_rate>10.0</update_rate>
                    <robot_name>robot</robot_name>
                    <use_navgoal_to_start>false</use_navgoal_to_start>
                    <global_frame_to_publish>map</global_frame_to_publish>
                    <ignore_models>
                        <model>visual</model>
                        <model>link</model>
                        <model>column</model>
                        <model>surface</model>
                    </ignore_models>
                </plugin>

            </actor>
        </sdf>"""

        return sdf 

    def _get_agent_height(self, skin_type: int) -> float:
        """Get correct height based on skin type"""
        heights = {
            0: 0.96,  # Elegant man
            1: 0.97,  # Casual man  
            2: 0.93,  # Elegant woman
            3: 0.93,  # Regular man
            4: 0.97,  # Worker man
            5: 1.05,  # Balds
            6: 1.05,
            7: 1.05,
            8: 1.05
        }
        return heights.get(skin_type, 1.0)




    def spawn_obstacles(self, obstacles):
        """Spawn static obstacles with debug logging"""
        self.node.get_logger().warn("=== SPAWN_OBSTACLES START ===")
        self.node.get_logger().warn(f"Attempting to spawn {len(list(obstacles))} obstacles")
        
        for obstacle in obstacles:
            self.node.get_logger().warn(f"\nProcessing obstacle: {obstacle.name}")
            
            # Known obstacles check
            self.node.get_logger().warn(f"Checking if {obstacle.name} is known")
            known = self._known_obstacles.get(obstacle.name)
            if known is not None:
                self.node.get_logger().warn(f"{obstacle.name} is already known")
                if known.obstacle.name != obstacle.name:
                    error_msg = f"New model name {obstacle.name} does not match model name {known.obstacle.name}"
                    self.node.get_logger().error(error_msg)
                    raise RuntimeError(error_msg)
                known.layer = ObstacleLayer.INUSE
            else:
                self.node.get_logger().warn(f"Creating new known obstacle for {obstacle.name}")
                known = self._known_obstacles.create_or_get(
                    name=obstacle.name,
                    obstacle=obstacle,
                    hunav_spawned=False,
                    layer=ObstacleLayer.INUSE,
                )

            # Attempt to spawn
            self.node.get_logger().warn(f"Spawning entity {obstacle.name}")
            success = self._simulator.spawn_entity(obstacle)
            if success:
                self.node.get_logger().warn(f"Successfully spawned {obstacle.name}")
            else:
                self.node.get_logger().error(f"Failed to spawn {obstacle.name}")

        self.node.get_logger().warn("=== SPAWN_OBSTACLES COMPLETE ===")




    def spawn_dynamic_obstacles(self, obstacles: typing.Collection[DynamicObstacle]):
        self.__logger.info(f"Attempting to spawn {len(list(obstacles))} dynamic obstacles")
        
        for obstacle in obstacles:

            try:
                
                hunav_obstacle = HunavDynamicObstacle.parse(obstacle.extra, obstacle.model)
 
                # Set name based on ID
                hunav_obstacle = attr.evolve(hunav_obstacle, name=f"agent{hunav_obstacle.id}")
                self.node.get_logger().warn(f"\nProcessing Agent with name: {hunav_obstacle.name}")

                velocity_twist = geometry_msgs.msg.Twist() #set velocity hardcoded like in old hunavgazebowrapper
                velocity_twist.linear.x = 0.6
                velocity_twist.linear.y = 0.0 
                velocity_twist.linear.z = 0.0
                velocity_twist.angular.x = 0.0
                velocity_twist.angular.y = 0.0
                velocity_twist.angular.z = 0.1
                
                hunav_obstacle = attr.evolve(hunav_obstacle, velocity=velocity_twist)

                # Check if obstacle is known to not register an agent multiple times unnecessarily
                known = self._known_obstacles.get(hunav_obstacle.name)
                if known is not None:
                    self.__logger.info(f"{hunav_obstacle.name} is already known and hunav_spawned: {known.hunav_spawned}")
                    if known.hunav_spawned:
                        self.__logger.info(f"Skipping spawn for {hunav_obstacle.name} as it's already registered")
                        continue
                    known.layer = ObstacleLayer.INUSE
                else:
                    self.__logger.info(f"Creating new known obstacle for {hunav_obstacle.name}")
                    known = self._known_obstacles.create_or_get(
                        name=hunav_obstacle.name,
                        obstacle=hunav_obstacle, 
                        hunav_spawned=False,
                        layer=ObstacleLayer.INUSE,
                    )

                # Register and spawn
                success = self.register_pedestrian(hunav_obstacle)
                if success:
                    known.hunav_spawned = True
                    self.__logger.info(f"Successfully registered and spawned {hunav_obstacle.name}")
                
            except Exception as e:
                self.__logger.error(f"Error processing obstacle: {str(e)}")
                traceback.print_exc()
    
    def register_pedestrian(self, hunav_obstacle: HunavDynamicObstacle) -> bool:
        """Register and spawn a HuNav agent"""
        self.__logger.info(f"HUNAVOBSTACLEREGISTER {hunav_obstacle}")
        try:
            entity_name = hunav_obstacle.name
            self.__logger.info(f"Registering pedestrian {entity_name}")
            
            # Check if already in simulator
            if entity_name in self._simulator.entities:
                self.__logger.warn(f"Entity {entity_name} already exists in simulator")
                return False
            # Create agent message from HunavDynamicObstacle
            agent_msg = Agent()
            agent_msg.id = hunav_obstacle.id
            agent_msg.name = hunav_obstacle.name
            agent_msg.type = hunav_obstacle.type
            agent_msg.skin = hunav_obstacle.skin
            agent_msg.group_id = hunav_obstacle.group_id
            agent_msg.velocity = hunav_obstacle.velocity
            agent_msg.desired_velocity = hunav_obstacle.desired_velocity
            agent_msg.linear_vel=hunav_obstacle.linear_vel
            agent_msg.angular_vel=hunav_obstacle.angular_vel
            agent_msg.radius = hunav_obstacle.radius

            # Set position
            agent_msg.position = geometry_msgs.msg.Pose()
            agent_msg.position.position.x = hunav_obstacle.position.x 
            agent_msg.position.position.y = hunav_obstacle.position.y 
            agent_msg.position.position.z = 1.250000 
            agent_msg.yaw = hunav_obstacle.position.orientation

            # Set behavior
            agent_msg.behavior = AgentBehavior()
            agent_msg.behavior.type = hunav_obstacle.behavior.type
            agent_msg.behavior.configuration = hunav_obstacle.behavior.configuration
            agent_msg.behavior.duration = hunav_obstacle.behavior.duration
            agent_msg.behavior.once = hunav_obstacle.behavior.once
            agent_msg.behavior.vel = hunav_obstacle.behavior.vel
            agent_msg.behavior.dist = hunav_obstacle.behavior.dist
            agent_msg.behavior.goal_force_factor = hunav_obstacle.behavior.goal_force_factor
            agent_msg.behavior.obstacle_force_factor = hunav_obstacle.behavior.obstacle_force_factor
            agent_msg.behavior.social_force_factor = hunav_obstacle.behavior.social_force_factor
            agent_msg.behavior.other_force_factor = hunav_obstacle.behavior.other_force_factor

            # Set goals
            agent_msg.goal_radius = hunav_obstacle.goal_radius
            agent_msg.cyclic_goals = hunav_obstacle.cyclic_goals
            # Add predefined goals if none exist
            if not hunav_obstacle.goals:
                goals = [
                    (-3.133759, -4.166653, 1.250000),
                    (0.997901, -4.131655, 1.250000),
                    (-0.227549, -20.187146, 1.250000)
                ]
                
                for x, y, h in goals:
                    goal = geometry_msgs.msg.Pose()
                    goal.position.x = x
                    goal.position.y = y
                    agent_msg.goals.append(goal)
            else:
                agent_msg.goals = hunav_obstacle.goals

            # Add wall obstacles
            self._update_agent_obstacles(agent_msg)

            # After creating the agent message:
            self.node.get_logger().warn(f"""            ##Complete Debug for the set attributes
            Full HunavObstacle Details:
            ID: {agent_msg.id}
            Name: {agent_msg.name}
            Type: {agent_msg.type}
            Skin: {agent_msg.skin}
            Group ID: {agent_msg.group_id}

            Position:
            - X: {agent_msg.position.position.x}
            - Y: {agent_msg.position.position.y} 
            - Z: {agent_msg.position.position.z}
            - Yaw: {agent_msg.yaw}

            Velocities:
            - Desired: {agent_msg.desired_velocity}
            - Linear: {agent_msg.linear_vel}
            - Angular: {agent_msg.angular_vel}

            Physical:
            - Radius: {agent_msg.radius}

            Behavior:
            - Type: {agent_msg.behavior.type}
            - Configuration: {agent_msg.behavior.configuration}
            - Duration: {agent_msg.behavior.duration}
            - Once: {agent_msg.behavior.once}
            - Velocity: {agent_msg.behavior.vel}
            - Distance: {agent_msg.behavior.dist}
            - Goal Force: {agent_msg.behavior.goal_force_factor}
            - Obstacle Force: {agent_msg.behavior.obstacle_force_factor}
            - Social Force: {agent_msg.behavior.social_force_factor}
            - Other Force: {agent_msg.behavior.other_force_factor}

            Goals:
            - Count: {len(agent_msg.goals)}
            - Cyclic: {agent_msg.cyclic_goals}
            - Radius: {agent_msg.goal_radius}
            - Goals List: {[f'({g.position.x}, {g.position.y})' for g in agent_msg.goals]}
            """)
            # Create agents message container
            peds = Agents()
            peds.header.stamp = self.node.get_clock().now().to_msg()
            peds.header.frame_id = "map"
            peds.agents.append(agent_msg)

            # Register with HuNav
            request = ComputeAgents.Request()
            request.robot = self._create_robot_message() # create a basic robot message to meet the message requirement
            request.current_agents = peds

            response = self._compute_agents_client.call(request)
            #self.node.get_logger().warn(f"############################  response (registering): { response}")
            
            if response:
                # Store in pedestrians dictionary
                self._pedestrians[agent_msg.id] = {
                    'last_update': time.time(),
                    'current_state': agent_msg.behavior.state,
                    'agent': agent_msg,
                    'animation_time': 0.0
                }
                self.__logger.info(f"self._pedestrians{self._pedestrians}")
                
                # Create and spawn visual model
                sdf = self.create_pedestrian_sdf(hunav_obstacle)
                
                spawn_obstacle = Obstacle(
                    position=hunav_obstacle.position,
                    name=entity_name,
                    model=ModelWrapper.from_model(Model(
                        type=ModelType.SDF,
                        name=entity_name,
                        description=sdf,
                        path=''
                    )),
                    extra=hunav_obstacle.extra
                )

                # Store and spawn
                self._simulator.entities[hunav_obstacle.name] = spawn_obstacle
                success = self._simulator.spawn_entity(spawn_obstacle)

                if success:
                    self.__logger.warn(f"Successfully spawned agent: {entity_name}")
                    return True

            return False

        except Exception as e:
            self.__logger.error(f"Error creating/registering agent: {str(e)}")
            self.__logger.error(traceback.format_exc())
            return False



    def _create_robot_message(self):
        """Creates a standard robot message for HuNav communication"""
        robot_msg = Agent()
        robot_msg.id = 0
        robot_msg.name = "robot"
        robot_msg.type = Agent.ROBOT
        robot_msg.position.position.x = 0.0
        robot_msg.position.position.y = 0.0
        robot_msg.yaw = 0.0
        robot_msg.radius = 0.3
        return robot_msg




    def spawn_walls(self, walls: WorldWalls, heightmap: WorldMap):
        """Spawn walls"""
        #self.node.get_logger().debug(f'spawning {len(walls)} walls')
        # Convert walls to obstacle and spawn it
        wall_obstacle = walls_to_obstacle(heightmap)
        self._simulator.spawn_entity(wall_obstacle)
        self._known_obstacles.create_or_get(
            name=wall_obstacle.name,
            obstacle=wall_obstacle,
            hunav_spawned=False,
            layer=ObstacleLayer.INUSE  # Change to INUSE from WORLD
        )


    def _update_agent_obstacles(self, agent_msg: Agent): 
        """Update agent's closest_obs with wall points"""
        wall_points = self._get_wall_points()
        agent_msg.closest_obs.extend(wall_points)

    def unuse_obstacles(self):
        self.__logger.debug(f'unusing obstacles')
        for obstacle_id, obstacle in self._known_obstacles.items():
            if obstacle.layer == ObstacleLayer.INUSE:
                obstacle.layer = ObstacleLayer.UNUSED

    def remove_obstacles(
            self, purge: ObstacleLayer = ObstacleLayer.UNUSED):
        self.__logger.debug(f'removing obstacles (level {purge})')

        for obstacle_id, obstacle in list(self._known_obstacles.items()):
            if purge >= obstacle.layer:
                self._known_obstacles.forget(name=obstacle_id)
                self._simulator.delete_entity(name=obstacle_id)


    def spawn_robot(self, robot: Robot):
        self.__logger.debug(f'spawning robot {robot.name}')
        self._simulator.spawn_entity(robot)

    def remove_robot(self, name: str):
        self.__logger.debug(f'removing robot {name}')
        self._simulator.delete_entity(name)

    def move_robot(self, name: str, position: PositionOrientation):
        self.__logger.debug(
            f'moving robot {name} to {repr(position)}')




    def _get_animation_factor(self, behavior: AgentBehavior) -> float:
        """Get animation factor with debug logging"""
        self.node.get_logger().warn(f"Getting animation factor for behavior type {behavior.type}")
        
        if behavior.state == AgentBehavior.BEH_NO_ACTIVE:
            self.node.get_logger().warn("Behavior inactive, returning default factor 1.0")
            return 1.0

        factor = 1.0
        if behavior.type == AgentBehavior.BEH_REGULAR:
            factor = 1.5
        elif behavior.type == AgentBehavior.BEH_IMPASSIVE:
            factor = 1.5
        elif behavior.type == AgentBehavior.BEH_SURPRISED:
            factor = 1.0
        elif behavior.type == AgentBehavior.BEH_THREATENING:
            factor = 1.0
        elif behavior.type == AgentBehavior.BEH_SCARED:
            factor = 1.5
        elif behavior.type == AgentBehavior.BEH_CURIOUS:
            factor = 1.0

        self.node.get_logger().warn(f"Returning animation factor: {factor}")
        return factor


  
    def parse_ped_type(self, t: str | int) -> int:
        """Convert various pedestrian type formats to HuNav's expected numeric types.
        
        Args:
            t: The type value, either as string or integer
            
        Returns:
            int: Corresponding HuNav agent type value
        """
        if isinstance(t, int):
            return t
        if isinstance(t, str):
            t = t.lower()
            if t in ('adult', 'elder', 'child'):
                return 1  # Agent.PERSON
            if t == 'robot':
                return 2  # Agent.ROBOT
        return 3  # Agent.OTHER


