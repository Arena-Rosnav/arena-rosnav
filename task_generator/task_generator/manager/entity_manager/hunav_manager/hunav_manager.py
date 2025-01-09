import dataclasses
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
from hunav_msgs.srv import (ComputeAgent, ComputeAgents, GetAgents, MoveAgent,
                            ResetAgents)

from task_generator.manager.entity_manager import EntityManager
from task_generator.manager.entity_manager.hunav_manager import HunavDynamicObstacle
from task_generator.manager.entity_manager.utils import (KnownObstacles,
                                                         ObstacleLayer,
                                                         walls_to_obstacle)
from task_generator.shared import (DynamicObstacle, Model, ModelType,
                                   Namespace, PositionOrientation, Robot, Obstacle)
from task_generator.simulators import BaseSimulator
from task_generator.utils.geometry import quaternion_from_euler
from task_generator.manager.world_manager.utils import WorldMap, WorldWalls
from .import SKIN_TYPES 


import dataclasses
import functools
import math
import os
import time
from threading import Lock
from typing import Any, Callable, Collection, Dict, List

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion
from hunav_msgs.msg import Agent, AgentBehavior, Agents
from hunav_msgs.srv import (ComputeAgent, ComputeAgents, GetAgents, MoveAgent,
                            ResetAgents)

from task_generator.manager.entity_manager import EntityManager
from task_generator.manager.entity_manager.hunav_manager import HunavDynamicObstacle
from task_generator.manager.entity_manager.utils import (KnownObstacles,
                                                         ObstacleLayer,
                                                         walls_to_obstacle)
from task_generator.shared import (DynamicObstacle, Model,ModelWrapper, ModelType,
                                   Namespace, PositionOrientation, Robot)
from task_generator.simulators import BaseSimulator
from task_generator.utils.geometry import quaternion_from_euler
import traceback


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

    def __init__(self, namespace: Namespace, simulator: BaseSimulator):
        """Initialize HunavManager with debug logging"""
        self.__logger = self.node.get_logger().get_child('hunav_EM')
        self.node.get_logger().warn("=== HUNAVMANAGER INIT START ===")
        super().__init__(namespace=namespace, simulator=simulator)
        self.node.get_logger().warn("Parent class initialized")

        # Initialize state variables
        self.node.get_logger().warn("Initializing state variables...")
        self._is_paused = False
        self._semaphore_reset = False
        self._agents_initialized = False
        self._robot_initialized = False
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


    def _update_agents(self):
        """Updates agent positions"""
        try:
            if not self._pedestrians:  # Skip if no pedestrians registered
                return
                
            with self._lock:
                # Create robot message
                robot_msg = Agent()
                robot_msg.id = 0
                robot_msg.name = "robot"
                robot_msg.type = Agent.ROBOT
                robot_msg.position.position.x = 0.0
                robot_msg.position.position.y = 0.0
                robot_msg.yaw = 0.0
                robot_msg.radius = 0.3

                # Create request
                request = ComputeAgents.Request()
                request.robot = robot_msg
                request.current_agents = self._get_current_agents()

                # Call service
                response = self._compute_agents_client.call(request)
                
                if response and response.updated_agents.agents:
                    for agent in response.updated_agents.agents:
                        # Update position
                        new_position = PositionOrientation(
                            x=agent.position.position.x,
                            y=agent.position.position.y,
                            orientation=agent.yaw
                        )
                        self._simulator.move_entity(str(agent.id), new_position)
                        
                        # Update internal state
                        if agent.id in self._pedestrians:
                            self._pedestrians[agent.id].update({
                                'last_update': self.node.get_clock().now(),
                                'current_state': agent.behavior.state
                            })

        except Exception as e:
            self.__logger.error(f"Error in agent update: {str(e)}")
            self.__logger.error(traceback.format_exc())



    def _get_current_agents(self) -> Agents:
        """Creates current agents message for service call"""
        current_agents = Agents()
        current_agents.header.stamp = self.node.get_clock().now().to_msg()
        current_agents.header.frame_id = "map"
        
        for agent_id, agent_data in self._pedestrians.items():
            agent = agent_data.get('agent')
            if agent:
                current_agents.agents.append(agent)
        
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
        """Create SDF description for pedestrian"""
        # Get skin type
        skin_type = self.SKIN_TYPES.get(agent_config.skin, 'casual_man.dae')
        
        # Get workspace root 
        def get_workspace_root():
            current_dir = os.path.abspath(__file__)
            workspace_root = current_dir
            while not workspace_root.endswith("arena4_ws"):
                workspace_root = os.path.dirname(workspace_root)
                
            if not workspace_root.endswith("arena4_ws"):
                raise ValueError("Could not find the 'arena4_ws' directory")
                
            return workspace_root
            
        # Construct mesh path
        mesh_path = os.path.join(
            get_workspace_root(),
            'src/deps/hunav/hunav_sim/hunav_rviz2_panel/meshes/models',
            skin_type
        )

        sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.6">
            <model name="{agent_config.name}">
                <static>false</static>
                <pose>0 0 0 0 0 0</pose>
                <link name="link">
                    <inertial>
                        <mass>70.0</mass>
                        <inertia>
                            <ixx>0.83</ixx>
                            <ixy>0.0</ixy>
                            <ixz>0.0</ixz>
                            <iyy>0.83</iyy>
                            <iyz>0.0</iyz>
                            <izz>0.083</izz>
                        </inertia>
                    </inertial>
                    <collision name="collision">
                        <pose>0 0 0.85 0 0 0</pose>
                        <geometry>
                            <cylinder>
                                <radius>{agent_config.radius}</radius>
                                <length>1.7</length>
                            </cylinder>
                        </geometry>
                    </collision>
                    <visual name="visual">
                        <pose>0 0 0.85 0 0 0</pose>
                        <geometry>
                            <mesh>
                                <uri>file://{mesh_path}</uri>
                                <scale>1.0 1.0 1.0</scale>
                            </mesh>
                        </geometry>
                    </visual>
                </link>
            </model>
        </sdf>"""
        
        return sdf





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



    
    
    def register_pedestrian(self) -> bool:
        """Create and register a single HuNav agent using default.yaml configuration"""
        try:
            # Create base agent message
            agent_msg = Agent()
            agent_msg.id = 1
            agent_msg.name = "agent1"
            agent_msg.type = Agent.PERSON
            agent_msg.skin = 0
            agent_msg.group_id = -1
            agent_msg.desired_velocity = 1.5
            agent_msg.radius = 0.4

            # Set initial position
            agent_msg.position = geometry_msgs.msg.Pose()
            agent_msg.position.position.x = -3.973340
            agent_msg.position.position.y = -8.576801
            agent_msg.position.position.z = 1.250000
            agent_msg.yaw = 0.0

            # Set behavior parameters
            agent_msg.behavior = AgentBehavior()
            agent_msg.behavior.type = 4  # SCARED behavior
            agent_msg.behavior.configuration = 0  # Default configuration
            agent_msg.behavior.duration = 40.0
            agent_msg.behavior.once = True
            agent_msg.behavior.vel = 0.6
            agent_msg.behavior.dist = 0.0
            agent_msg.behavior.goal_force_factor = 2.0
            agent_msg.behavior.obstacle_force_factor = 10.0
            agent_msg.behavior.social_force_factor = 5.0
            agent_msg.behavior.other_force_factor = 20.0

            # Set goals
            agent_msg.goal_radius = 0.3
            agent_msg.cyclic_goals = True
            
            # Add predefined goals
            goals = [
                (-3.133759, -4.166653, 1.250000),
                (0.997901, -4.131655, 1.250000),
                (-0.227549, -10.187146, 1.250000)
            ]
            
            for x, y, h in goals:
                goal = geometry_msgs.msg.Pose()
                goal.position.x = x
                goal.position.y = y
                agent_msg.goals.append(goal)

            # Create agents message container
            peds = Agents()
            peds.header.stamp = self.node.get_clock().now().to_msg()
            peds.header.frame_id = "map"
            peds.agents.append(agent_msg)

            # Create robot message
            robot_msg = Agent()
            robot_msg.id = 0
            robot_msg.name = "robot"
            robot_msg.type = Agent.ROBOT
            robot_msg.position.position.x = 0.0
            robot_msg.position.position.y = 0.0
            robot_msg.yaw = 0.0
            robot_msg.radius = 0.3

            # Register with HuNav
            request = ComputeAgents.Request()
            request.robot = robot_msg
            request.current_agents = peds

            response = self._compute_agents_client.call(request)
            
            if response:
                self._pedestrians[agent_msg.id] = {
                    'last_update': time.time(),
                    'current_state': agent_msg.behavior.state,
                    'agent': agent_msg  # Store the complete configuration
                }
                
                # Create and spawn visual model
                sdf = self.create_pedestrian_sdf(agent_msg)
                spawn_model = Model(
                    description=sdf,
                    type=ModelType.SDF,
                    name=str(agent_msg.id),
                    path=''
                )
                
                # Create HunavDynamicObstacle for spawning
                spawn_position = PositionOrientation(
                    x=agent_msg.position.position.x,
                    y=agent_msg.position.position.y,
                    orientation=agent_msg.yaw
                )
                
                spawn_obstacle = HunavDynamicObstacle(
                    position=spawn_position,
                    name=agent_msg.name,
                    model=ModelWrapper.from_model(spawn_model),
                    extra={},
                    waypoints=[],
                    id=agent_msg.id,
                    type=agent_msg.type,
                    skin=agent_msg.skin,
                    group_id=agent_msg.group_id,
                    yaw=agent_msg.yaw,
                    velocity=None,
                    desired_velocity=agent_msg.desired_velocity,
                    radius=agent_msg.radius,
                    linear_vel=agent_msg.linear_vel,
                    angular_vel=agent_msg.angular_vel,
                    behavior=HunavDynamicObstacle.Behavior(
                        type=agent_msg.behavior.type,
                        state=agent_msg.behavior.state,
                        configuration=agent_msg.behavior.configuration,
                        duration=agent_msg.behavior.duration,
                        once=agent_msg.behavior.once,
                        vel=agent_msg.behavior.vel,
                        dist=agent_msg.behavior.dist,
                        social_force_factor=agent_msg.behavior.social_force_factor,
                        goal_force_factor=agent_msg.behavior.goal_force_factor,
                        obstacle_force_factor=agent_msg.behavior.obstacle_force_factor,
                        other_force_factor=agent_msg.behavior.other_force_factor
                    ),
                    goals=agent_msg.goals,
                    cyclic_goals=agent_msg.cyclic_goals,
                    goal_radius=agent_msg.goal_radius,
                    closest_obs=[]
                )
                
                return self._simulator.spawn_entity(spawn_obstacle)

            return False

        except Exception as e:
            self.__logger.error(f"Error creating/registering agent: {str(e)}")
            self.__logger.error(traceback.format_exc())
            return False



    def spawn_dynamic_obstacles(self, obstacles: typing.Collection[DynamicObstacle]):
        """Spawn single HuNav agent"""
        self.__logger.info("Creating and registering HuNav agent")
        success = self.register_pedestrian()
        if not success:
            self.__logger.error("Failed to create and register agent")


    def spawn_walls(self, walls: WorldWalls, heightmap: WorldMap):
        """Spawn walls"""
        self.node.get_logger().debug(f'spawning {len(walls)} walls')
        # Convert walls to obstacle and spawn it
        wall_obstacle = walls_to_obstacle(heightmap)
        self._simulator.spawn_entity(wall_obstacle)
        self._known_obstacles.create_or_get(
            name=wall_obstacle.name,
            obstacle=wall_obstacle,
            hunav_spawned=False,
            layer=ObstacleLayer.INUSE  # Change to INUSE from WORLD
        )


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
































    def _update_pedestrians(self):
        """Update pedestrians with debug logging"""
        with self._lock:
            self.node.get_logger().warn("=== UPDATE_PEDESTRIANS START ===")
            current_time = time.time()

            # Get updates from Hunav
            self.node.get_logger().warn("Requesting agent updates from HuNav")
            request = ComputeAgents.Request()
            future = self._compute_agents_client.call(request)

            if future.result() is not None:
                agents = future.result().agents
                self.node.get_logger().warn(f"Received updates for {len(agents)} agents")
                
                for agent in agents:
                    if agent.id in self._pedestrians:
                        self.node.get_logger().warn(f"Updating pedestrian {agent.id}")
                        ped_data = self._pedestrians[agent.id]

                        # Update position
                        pos = PositionOrientation(
                            x=agent.position.position.x,
                            y=agent.position.position.y,
                            orientation=agent.yaw
                        )
                        self.node.get_logger().warn(f"Moving entity {agent.id} to new position")
                        self._simulator.move_entity(name=agent.id, position=pos)

                        # Update animation
                        dt = current_time - ped_data['last_update']
                        animation_factor = self._get_animation_factor(agent.behavior)
                        ped_data['animation_time'] += dt * animation_factor

                        # Update animation state if needed
                        if agent.behavior.state != ped_data.get('current_state'):
                            self.node.get_logger().warn(f"Updating animation state for {agent.id}")
                            self._update_agent_animation(agent.id, agent.behavior)
                            ped_data['current_state'] = agent.behavior.state

                        ped_data['last_update'] = current_time
                    else:
                        self.node.get_logger().warn(f"Agent {agent.id} not found in pedestrians")

            self.node.get_logger().warn("=== UPDATE_PEDESTRIANS COMPLETE ===")

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

    def _update_agent_animation(self, agent_id: str, behavior: AgentBehavior):
        """Update agent animation with debug logging"""
        self.node.get_logger().warn(f"=== UPDATE_ANIMATION for agent {agent_id} ===")
        
        animation = 'WALK'
        if behavior.state == AgentBehavior.BEH_NO_ACTIVE:
            animation = 'NORMAL_WAIT'
        else:
            if behavior.type == AgentBehavior.BEH_REGULAR:
                animation = 'WALK'
            elif behavior.type == AgentBehavior.BEH_IMPASSIVE:
                animation = 'WALK_FORWARD'
            elif behavior.type == AgentBehavior.BEH_SURPRISED:
                animation = 'NORMAL_WAIT'
            elif behavior.type == AgentBehavior.BEH_THREATENING:
                animation = 'WALK_ANGRY'
            elif behavior.type == AgentBehavior.BEH_SCARED:
                animation = 'WALK_SCARED'
            elif behavior.type == AgentBehavior.BEH_CURIOUS:
                animation = 'SLOW_WALK'

        self.node.get_logger().warn(f"Selected animation: {animation}")
        if agent_id in self._pedestrians:
            self._pedestrians[agent_id]['current_animation'] = animation
            self.node.get_logger().warn(f"Animation updated for agent {agent_id}")
        else:
            self.node.get_logger().warn(f"Agent {agent_id} not found in pedestrians")


    def _load_agent_config(self, agent_id: str) -> Dict[str, Any]:
        """Load configuration for a specific agent

        Args:
            agent_id (str): ID of the agent

        Returns:
            Dict[str, Any]: Configuration dictionary
        """
        if agent_id not in self.agent_config:
            self.node.get_logger().warn(
                f"No configuration found for agent {agent_id}")
            return {}
        return self.agent_config[agent_id]




    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle with debug logging"""
        normalized = math.fmod(angle, math.pi)
        self.node.get_logger().warn(f"Normalized angle from {angle} to {normalized}")
        return normalized

  
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


