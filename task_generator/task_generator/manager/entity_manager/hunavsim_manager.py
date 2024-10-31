import dataclasses
import functools
import time
import os
import yaml
from typing import Callable, List, Collection, Dict, Any
from threading import Lock

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from hunav_msgs.srv import ComputeAgent, ComputeAgents, GetAgents, MoveAgent, ResetAgents
from hunav_msgs.msg import Agent, Agents, AgentBehavior
from std_srvs.srv import Empty, Trigger
from ament_index_python.packages import get_package_share_directory

from task_generator.constants import Constants
from task_generator.constants.runtime import Config
from task_generator.manager.entity_manager.entity_manager import EntityManager
from task_generator.manager.entity_manager.utils import (
    KnownObstacles,
    ObstacleLayer,
    SDFUtil,
    walls_to_obstacle,
)
from task_generator.shared import (
    DynamicObstacle,
    Model,
    ModelType,
    Namespace,
    Obstacle,
    PositionOrientation,
    Robot,
)
from task_generator.simulators.gazebo_simulator import GazeboSimulator
from task_generator.utils.geometry import quaternion_from_euler, euler_from_quaternion

class HunavsimManager(EntityManager):
    # Animation configuration (from WorldGenerator)
    SKIN_TYPES = {
        0: 'elegant_man.dae',
        1: 'casual_man.dae',
        2: 'elegant_woman.dae',
        3: 'regular_man.dae',
        4: 'worker_man.dae',
        5: 'walk.dae'
    }
    
    ANIMATION_TYPES = {
        'WALK': '07_01-walk.bvh',
        'WALK_FORWARD': '69_02_walk_forward.bvh',
        'NORMAL_WAIT': '137_28-normal_wait.bvh',
        'WALK_CHILDISH': '142_01-walk_childist.bvh',
        'SLOW_WALK': '07_04-slow_walk.bvh',
        'WALK_SCARED': '142_17-walk_scared.bvh',
        'WALK_ANGRY': '17_01-walk_with_anger.bvh'
    }

    # Service Names
    SERVICE_COMPUTE_AGENT = 'compute_agent'
    SERVICE_COMPUTE_AGENTS = 'compute_agents'
    SERVICE_GET_AGENTS = 'get_agents'
    SERVICE_MOVE_AGENT = 'move_agent'
    SERVICE_RESET_AGENTS = 'reset_agents'
    
    def __init__(self, namespace: Namespace, simulator: GazeboSimulator, node: Node = None):
        super().__init__(namespace=namespace, simulator=simulator)
        
        self._node = node
        if self._node is None:
            from task_generator import TASKGEN_NODE
            self._node = TASKGEN_NODE
            
        # Initialize state variables
        self._is_paused = False
        self._semaphore_reset = False
        self._agents_initialized = False
        self._robot_initialized = False
        self._lock = Lock()
        self._update_rate = 0.1  # 10Hz update rate
        
        # Load agent configuration
        self.load_agent_configurations()
        
        # Initialize service clients
        self.setup_services()
        
        # Setup timer for pedestrian updates (like HuNavPlugin's OnUpdate)
        self._update_timer = self._node.create_timer(
            self._update_rate,
            self.update_pedestrians
        )
        
        # Initialize JAIL_POS generator
        def gen_JAIL_POS(steps: int, x: int = 1, y: int = 0):
            steps = max(steps, 1)
            while True:
                x += y == steps
                y %= steps
                yield PositionOrientation(-x, y, 0)
                y += 1
        self.JAIL_POS = gen_JAIL_POS(10)

    def load_agent_configurations(self):
        """Load agent configurations from YAML (WorldGenerator functionality)"""
        config_path = os.path.join(
            get_package_share_directory('arena_bringup'),
            'configs',
            'hunav_agents',
            'default.yaml'
        )
        with open(config_path, 'r') as f:
            self.agent_config = yaml.safe_load(f)['hunav_loader']['ros__parameters']
            
        self._known_obstacles = KnownObstacles()
        self._pedestrians = {}  # Store pedestrian states
        
        # Process configurations like WorldGenerator
        for agent_name, config in self.agent_config.items():
            if isinstance(config, dict) and 'id' in config:
                self._pedestrians[agent_name] = {
                    'config': config,
                    'current_animation': 'WALK',
                    'animation_time': 0.0,
                    'last_update': time.time()
                }

    def setup_services(self):
        """Initialize all required services"""
        self._compute_agent_client = self._node.create_client(
            ComputeAgent,
            self._namespace(self.SERVICE_COMPUTE_AGENT)
        )
        self._compute_agents_client = self._node.create_client(
            ComputeAgents,
            self._namespace(self.SERVICE_COMPUTE_AGENTS)
        )
        self._get_agents_client = self._node.create_client(
            GetAgents,
            self._namespace(self.SERVICE_GET_AGENTS)
        )
        self._move_agent_client = self._node.create_client(
            MoveAgent,
            self._namespace(self.SERVICE_MOVE_AGENT)
        )
        self._reset_agents_client = self._node.create_client(
            ResetAgents,
            self._namespace(self.SERVICE_RESET_AGENTS)
        )
        
        # Wait for services
        required_services = [
            (self._compute_agent_client, 'compute_agent'),
            (self._compute_agents_client, 'compute_agents'),
            (self._get_agents_client, 'get_agents'),
            (self._move_agent_client, 'move_agent'),
            (self._reset_agents_client, 'reset_agents')
        ]
        
        for client, name in required_services:
            while not client.wait_for_service(timeout_sec=1.0):
                self._node.get_logger().info(f'{name} service not available, waiting...')

    def create_pedestrian_sdf(self, agent_config: Dict) -> str:
        """Create SDF description for pedestrian (from WorldGenerator)"""
        skin_type = self.SKIN_TYPES.get(agent_config.get('skin', 0), 'casual_man.dae')
        
        # Get path to mesh file
        mesh_path = os.path.join(
            get_package_share_directory('hunav_sim'),
            'hunav_rviz2_panel/meshes',
            skin_type
        )
        
        # Height adjustment based on skin type (from HuNavPlugin)
        height_adjustments = {
            'elegant_man.dae': 0.96,
            'casual_man.dae': 0.97,
            'elegant_woman.dae': 0.93,
            'regular_man.dae': 0.93,
            'worker_man.dae': 0.97
        }
        z_pos = height_adjustments.get(skin_type, 1.0)
        
        sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.6">
            <model name="{agent_config['name']}">
                <static>false</static>
                <pose>0 0 {z_pos} 0 0 0</pose>
                <link name="link">
                    <visual name="visual">
                        <geometry>
                            <mesh>
                                <uri>file://{mesh_path}</uri>
                            </mesh>
                        </geometry>
                    </visual>
                    <collision name="collision">
                        <geometry>
                            <cylinder>
                                <radius>0.3</radius>
                                <length>1.7</length>
                            </cylinder>
                        </geometry>
                    </collision>
                </link>
            </model>
        </sdf>"""
        return sdf

    def spawn_dynamic_obstacles(self, obstacles: Collection[DynamicObstacle]):
        """Spawn dynamic obstacles/agents"""
        for obstacle in obstacles:
            # Load agent configuration
            agent_config = self._load_agent_config(obstacle.name)
            
            # Create Hunav Agent
            request = ComputeAgent.Request()
            agent = Agent()
            agent.id = obstacle.name
            agent.type = agent.PERSON
            
            # Set behavior
            agent.behavior = AgentBehavior()
            behavior = agent_config.get('behavior', {})
            agent.behavior.type = behavior.get('type', 1)
            agent.behavior.configuration = behavior.get('configuration', 0)
            agent.behavior.duration = behavior.get('duration', 40.0)
            agent.behavior.once = behavior.get('once', True)
            agent.behavior.vel = behavior.get('vel', 0.6)
            agent.behavior.dist = behavior.get('dist', 0.0)
            agent.behavior.goal_force_factor = behavior.get('goal_force_factor', 2.0)
            agent.behavior.obstacle_force_factor = behavior.get('obstacle_force_factor', 10.0)
            agent.behavior.social_force_factor = behavior.get('social_force_factor', 5.0)
            agent.behavior.other_force_factor = behavior.get('other_force_factor', 20.0)
            
            # Set position and goals
            init_pose = agent_config.get('init_pose', {})
            agent.position.position = Point(
                x=init_pose.get('x', 0.0),
                y=init_pose.get('y', 0.0),
                z=init_pose.get('z', 0.0)
            )
            agent.yaw = init_pose.get('h', 0.0)
            
            # Add goals
            goals = agent_config.get('goals', [])
            for goal_id in goals:
                goal = agent_config.get(goal_id, {})
                goal_pose = Pose()
                goal_pose.position = Point(
                    x=goal.get('x', 0.0),
                    y=goal.get('y', 0.0),
                    z=goal.get('z', 0.0)
                )
                agent.goals.append(goal_pose)
            
            # Create SDF model (WorldGenerator functionality)
            sdf = self.create_pedestrian_sdf(agent_config)
            
            # Create model with SDF
            obstacle = dataclasses.replace(
                obstacle,
                model=Model(
                    description=sdf,
                    model_type=ModelType.SDF,
                    name=obstacle.name
                )
            )
            
            # Spawn in Gazebo
            self._simulator.spawn_entity(obstacle)
            
            # Register with Hunav
            request.agent = agent
            future = self._compute_agent_client.call_async(request)
            rclpy.spin_until_future_complete(self._node, future)
            
            known = self._known_obstacles.create_or_get(
                name=obstacle.name,
                obstacle=obstacle,
                hunav_spawned=True,
                layer=ObstacleLayer.INUSE,
            )

    def update_pedestrians(self):
        """Update pedestrians (from HuNavPlugin's OnUpdate)"""
        with self._lock:
            current_time = time.time()
            
            # Get updates from Hunav
            request = ComputeAgents.Request()
            future = self._compute_agents_client.call_async(request)
            rclpy.spin_until_future_complete(self._node, future)
            
            if future.result() is not None:
                agents = future.result().agents
                for agent in agents:
                    if agent.id in self._pedestrians:
                        ped_data = self._pedestrians[agent.id]
                        
                        # Update position
                        pos = PositionOrientation(
                            x=agent.position.position.x,
                            y=agent.position.position.y,
                            orientation=agent.yaw
                        )
                        self._simulator.move_entity(name=agent.id, position=pos)
                        
                        # Update animation (like HuNavPlugin)
                        dt = current_time - ped_data['last_update']
                        animation_factor = self.get_animation_factor(agent.behavior)
                        ped_data['animation_time'] += dt * animation_factor
                        
                        # Update animation state if needed
                        if agent.behavior.state != ped_data.get('current_state'):
                            self.update_agent_animation(agent.id, agent.behavior)
                            ped_data['current_state'] = agent.behavior.state
                        
                        ped_data['last_update'] = current_time

    def get_animation_factor(self, behavior: AgentBehavior) -> float:
        """Get animation speed factor based on behavior (from HuNavPlugin)"""
        if behavior.state == AgentBehavior.BEH_NO_ACTIVE:
            return 1.0
            
        if behavior.type == AgentBehavior.BEH_REGULAR:
            return 1.5
        elif behavior.type == AgentBehavior.BEH_IMPASSIVE:
            return 1.5
        elif behavior.type == AgentBehavior.BEH_SURPRISED:
            return 1.0
        elif behavior.type == AgentBehavior.BEH_THREATENING:
            return 1.0
        elif behavior.type == AgentBehavior.BEH_SCARED:
            return 1.5
        elif behavior.type == AgentBehavior.BEH_CURIOUS:
            return 1.0
        
        return 1.0

    def update_agent_animation(self, agent_id: str, behavior: AgentBehavior):
        """Update agent animation based on behavior (from HuNavPlugin)"""
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
            else:
                animation = 'WALK'
                
        if agent_id in self._pedestrians:
            self._pedestrians[agent_id]['current_animation'] = animation


    def _load_agent_config(self, agent_id: str) -> Dict[str, Any]:
            """Load configuration for a specific agent
            
            Args:
                agent_id (str): ID of the agent
                
            Returns:
                Dict[str, Any]: Configuration dictionary
            """
            if agent_id not in self.agent_config:
                self._node.get_logger().warn(f"No configuration found for agent {agent_id}")
                return {}
            return self.agent_config[agent_id]

    def spawn_obstacles(self, obstacles):
        """Spawn static obstacles"""
        for obstacle in obstacles:
            known = self._known_obstacles.get(obstacle.name)
            if known is not None:
                if known.obstacle.name != obstacle.name:
                    raise RuntimeError(
                        f"New model name {obstacle.name} does not match model name {known.obstacle.name} of known obstacle {obstacle.name}"
                    )
                known.layer = ObstacleLayer.INUSE
            else:
                known = self._known_obstacles.create_or_get(
                    name=obstacle.name,
                    obstacle=obstacle,
                    hunav_spawned=False,
                    layer=ObstacleLayer.INUSE,
                )
            
            self._simulator.spawn_entity(obstacle)

    def spawn_walls(self, walls, heightmap):
        """Spawn walls in simulation"""
        if self.WALLS_ENTITY in self._known_obstacles:
            return

        obstacle = walls_to_obstacle(heightmap)
        self._known_obstacles.create_or_get(
            name=self.WALLS_ENTITY,
            obstacle=obstacle,
            layer=ObstacleLayer.WORLD,
            hunav_spawned=False,
        )
        
        self._simulator.spawn_entity(obstacle)

    def remove_obstacles(self, purge):
        """Remove obstacles based on purge level"""
        if not self._is_paused:
            self._is_paused = True
            request = ResetAgents.Request()
            future = self._reset_agents_client.call_async(request)
            rclpy.spin_until_future_complete(self._node, future)

        while self._semaphore_reset:
            time.sleep(0.1)

        actions: List[Callable] = []
        self._semaphore_reset = True

        try:
            to_forget: List[str] = list()

            for obstacle_id, obstacle in self._known_obstacles.items():
                if purge >= obstacle.layer:
                    if isinstance(self._simulator, GazeboSimulator):
                        if isinstance(obstacle.obstacle, DynamicObstacle):
                            def move_to_jail(obstacle_id):
                                jail = next(self.JAIL_POS)
                                self._simulator.move_entity(name=obstacle_id, position=jail)
                            actions.append(functools.partial(move_to_jail, obstacle_id))
                        else:
                            def delete_entity(obstacle_id):
                                obstacle.hunav_spawned = False
                                self._simulator.delete_entity(name=obstacle_id)
                            actions.append(functools.partial(delete_entity, obstacle_id))
                            to_forget.append(obstacle_id)
                    else:
                        obstacle.hunav_spawned = False
                        to_forget.append(obstacle_id)

            for obstacle_id in to_forget:
                self._known_obstacles.forget(name=obstacle_id)
                if obstacle_id in self._pedestrians:
                    del self._pedestrians[obstacle_id]

        finally:
            self._semaphore_reset = False

        for action in actions:
            action()

    def move_robot(self, name: str, position: PositionOrientation):
        """Move robot to new position"""
        request = MoveAgent.Request()
        request.id = name
        request.pose.position = Point(x=position.x, y=position.y, z=0.0)
        quat = quaternion_from_euler(0.0, 0.0, position.orientation)
        request.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        future = self._move_agent_client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)

    def spawn_robot(self, robot: Robot):
        """Spawn robot in simulation"""
        self._simulator.spawn_entity(robot)

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi] (from HuNavPlugin)"""
        while angle <= -3.14159:
            angle += 2 * 3.14159
        while angle > 3.14159:
            angle -= 2 * 3.14159
        return angle