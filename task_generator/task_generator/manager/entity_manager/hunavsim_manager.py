import dataclasses
import functools
import time
import os
import yaml
from typing import Callable, List, Collection, Dict, Any

import rclpy
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
    # Service Names
    SERVICE_COMPUTE_AGENT = 'compute_agent'
    SERVICE_COMPUTE_AGENTS = 'compute_agents'
    SERVICE_GET_AGENTS = 'get_agents'
    SERVICE_MOVE_AGENT = 'move_agent'
    SERVICE_RESET_AGENTS = 'reset_agents'
    
    def __init__(self, namespace: Namespace, simulator: GazeboSimulator):
        """Initialize HunavsimManager
        
        Args:
            namespace (Namespace): Global namespace
            simulator (GazeboSimulator): Instance of GazeboSimulator
        """
        super().__init__(namespace=namespace, simulator=simulator)

        # Load agent configuration
        config_path = os.path.join(
            get_package_share_directory('arena_bringup'),
            'configs',
            'hunav_agents',
            'default.yaml'
        )
        with open(config_path, 'r') as f:
            self.agent_config = yaml.safe_load(f)['hunav_loader']['ros__parameters']

        self._known_obstacles = KnownObstacles()
        
        # Initialize service clients
        self._compute_agent_client = self.create_client(
            ComputeAgent,
            self._namespace(self.SERVICE_COMPUTE_AGENT)
        )
        self._compute_agents_client = self.create_client(
            ComputeAgents,
            self._namespace(self.SERVICE_COMPUTE_AGENTS)
        )
        self._get_agents_client = self.create_client(
            GetAgents,
            self._namespace(self.SERVICE_GET_AGENTS)
        )
        self._move_agent_client = self.create_client(
            MoveAgent,
            self._namespace(self.SERVICE_MOVE_AGENT)
        )
        self._reset_agents_client = self.create_client(
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
                self.get_logger().info(f'{name} service not available, waiting...')

        # Initialize state variables
        self._is_paused = False
        self._semaphore_reset = False
        self._agents_initialized = False
        self._robot_initialized = False

        # Initialize JAIL_POS generator for handling deleted actors
        def gen_JAIL_POS(steps: int, x: int = 1, y: int = 0):
            steps = max(steps, 1)
            while True:
                x += y == steps
                y %= steps
                yield PositionOrientation(-x, y, 0)
                y += 1
        self.JAIL_POS = gen_JAIL_POS(10)

    def create_pedestrian_sdf(self, name: str, skin_type: str) -> str:
        """Create SDF model description for a pedestrian
        
        Args:
            name: Name of the pedestrian
            skin_type: Type of mesh to use (casual_man, elegant_man, etc.)
        """
        # Get path to mesh file
        mesh_path = os.path.join(
            get_package_share_directory('hunav_sim'),
            'hunav_rviz2_panel/meshes',
            f'{skin_type}.dae'
        )
        
        sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.6">
            <model name="{name}">
                <static>false</static>
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

    def get_skin_type(self, skin_id: int) -> str:
        """Get mesh filename based on skin ID"""
        skin_types = {
            0: 'elegant_man',
            1: 'casual_man',
            2: 'elegant_woman',
            3: 'regular_man',
            4: 'worker_man'
        }
        return skin_types.get(skin_id, 'casual_man')

    def spawn_dynamic_obstacles(self, obstacles: Collection[DynamicObstacle]):
        """Spawn dynamic obstacles/agents using Hunav
        
        Args:
            obstacles (Collection[DynamicObstacle]): Collection of dynamic obstacles to spawn
        """
        for obstacle in obstacles:
            # Load agent configuration
            agent_config = self._load_agent_config(obstacle.name)
            
            # Create Hunav Agent
            agent = Agent()
            agent.id = obstacle.name
            agent.type = agent.PERSON
            
            # Set behavior
            agent.behavior = AgentBehavior()
            behavior = agent_config.get('behavior', {})
            agent.behavior.type = behavior.get('type', 1)  # REGULAR default
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
            
            # Create and spawn visual model in Gazebo
            skin_type = self.get_skin_type(agent_config.get('skin', 1))
            sdf = self.create_pedestrian_sdf(obstacle.name, skin_type)
            
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
            request = ComputeAgent.Request()
            request.agent = agent
            future = self._compute_agent_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            known = self._known_obstacles.create_or_get(
                name=obstacle.name,
                obstacle=obstacle,
                hunav_spawned=True,  # Already spawned above
                layer=ObstacleLayer.INUSE,
            )

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

    def move_robot(self, name: str, position: PositionOrientation):
        """Move robot to new position"""
        request = MoveAgent.Request()
        request.id = name
        request.pose.position = Point(x=position.x, y=position.y, z=0.0)
        quat = quaternion_from_euler(0.0, 0.0, position.orientation)
        request.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        future = self._move_agent_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def remove_obstacles(self, purge):
        """Remove obstacles based on purge level"""
        if not self._is_paused:
            self._is_paused = True
            request = ResetAgents.Request()
            future = self._reset_agents_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

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

        finally:
            self._semaphore_reset = False

        for action in actions:
            action()

    def spawn_robot(self, robot: Robot):
        """Spawn robot in simulation"""
        self._simulator.spawn_entity(robot)

    def update_agent_positions(self):
        """Update agent positions based on Hunav calculations"""
        request = ComputeAgents.Request()
        future = self._compute_agents_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            for agent in future.result().agents:
                if agent.id in self._known_obstacles:
                    pos = PositionOrientation(
                        x=agent.position.position.x,
                        y=agent.position.position.y,
                        orientation=agent.yaw
                    )
                    self._simulator.move_entity(name=agent.id, position=pos)

    def _load_agent_config(self, agent_id: str) -> Dict[str, Any]:
        """Load configuration for a specific agent
        
        Args:
            agent_id (str): ID of the agent
            
        Returns:
            Dict[str, Any]: Configuration dictionary
        """
        if agent_id not in self.agent_config:
            self.get_logger().warn(f"No configuration found for agent {agent_id}")
            return {}
        return self.agent_config[agent_id]