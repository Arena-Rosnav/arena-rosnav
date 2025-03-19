import dataclasses
import functools
import math
import os
import time
from threading import Lock
from typing import Any, Callable, Collection, Dict, List

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from hunav_msgs.msg import Agent, AgentBehavior, Agents
from hunav_msgs.srv import (ComputeAgent, ComputeAgents, MoveAgent, ResetAgents)

from task_generator.manager.entity_manager import EntityManager
from task_generator.manager.entity_manager.hunav_manager import HunavDynamicObstacle
from task_generator.manager.entity_manager.utils import (KnownObstacles,
                                                         ObstacleLayer,
                                                         walls_to_obstacle)
from task_generator.shared import (DynamicObstacle, Model, ModelType,
                                   Namespace, PositionOrientation, Robot)
from task_generator.simulators import BaseSimulator


class HunavManager(EntityManager):
    # Class constants
    WALLS_ENTITY = "walls"  # Definition for walls_entity

    # Service Names
    SERVICE_COMPUTE_AGENT = 'compute_agent'
    SERVICE_COMPUTE_AGENTS = 'compute_agents'
    # SERVICE_GET_AGENTS = 'get_agents' # does not actually exist
    SERVICE_MOVE_AGENT = 'move_agent'
    SERVICE_RESET_AGENTS = 'reset_agents'

    def __init__(self, namespace: Namespace, simulator: BaseSimulator):

        super().__init__(namespace=namespace, simulator=simulator)

        # Initialize state variables
        self._is_paused = False
        self._semaphore_reset = False
        self._agents_initialized = False
        self._robot_initialized = False
        self._lock = Lock()
        self._update_rate = 0.1

        # Initialize collections
        self._known_obstacles = KnownObstacles()  # Initialization
        self._pedestrians = {}  # Store pedestrian states

        # Setup services
        self.setup_services()

        # Wait to be sure that all services are ready
        time.sleep(2.0)

        # Service Test for Hunavsim
        # self.node.get_logger().info("Starting initial service tests...")
        # self.test_hunav_services()
        # self.node.get_logger().info("Initial service tests completed.")

        # Setup timer for pedestrian updates
        self._update_timer = self.node.create_timer(
            self._update_rate,
            self._update_pedestrians
        )

        # Setup timer for pedestrian updates
        self._update_timer = self.node.create_timer(
            self._update_rate,
            self._update_pedestrians
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

    def test_hunav_services(self):
        """Test all HuNav services with proper agent initialization"""
        self.node.get_logger().info("\n========= STARTING HUNAV SERVICES TEST =========")

        # Create test agents
        test_agents = Agents()
        test_agents.header.stamp = self.node.get_clock().now().to_msg()
        test_agents.header.frame_id = "map"

        # Create a test agent
        test_agent = Agent()
        test_agent.id = 1
        test_agent.name = "test_pedestrian"
        test_agent.type = Agent.PERSON
        test_agent.position.position.x = 2.0
        test_agent.position.position.y = 2.0
        test_agent.yaw = 0.0
        test_agent.desired_velocity = 1.0
        test_agent.radius = 0.35

        # Set behavior
        test_agent.behavior = AgentBehavior()
        test_agent.behavior.type = AgentBehavior.BEH_REGULAR
        test_agent.behavior.configuration = AgentBehavior.BEH_CONF_DEFAULT
        test_agent.behavior.duration = 40.0
        test_agent.behavior.once = True
        test_agent.behavior.goal_force_factor = 2.0
        test_agent.behavior.obstacle_force_factor = 10.0
        test_agent.behavior.social_force_factor = 5.0

        # Add test goal
        goal = Pose()
        goal.position.x = 5.0
        goal.position.y = 5.0
        test_agent.goals.append(goal)
        test_agent.cyclic_goals = True
        test_agent.goal_radius = 0.3

        # Add agent to agents message
        test_agents.agents.append(test_agent)

        # Create test robot
        test_robot = Agent()
        test_robot.id = 0
        test_robot.name = "test_robot"
        test_robot.type = Agent.ROBOT
        test_robot.position.position.x = 0.0
        test_robot.position.position.y = 0.0
        test_robot.yaw = 0.0
        test_robot.radius = 0.3

        self.node.get_logger().info("Created test agents and robot for service testing")

        # 1. Test compute_agents service
        try:
            self.node.get_logger().info("\n--- Testing /compute_agents service ---")
            request = ComputeAgents.Request()
            request.robot = test_robot
            request.current_agents = test_agents

            self.node.get_logger().info(
                f"Sending compute_agents request with {len(request.current_agents.agents)} agents")

            future = self._compute_agents_client.call(request)
            rclpy.spin_until_future_complete(self.node, future)

            if future.result():
                response = future.result()
                self.node.get_logger().info(
                    f"Received response with {len(response.updated_agents.agents)} agents")
                for agent in response.updated_agents.agents:
                    self.node.get_logger().info(
                        f"\nAgent {agent.name} (ID: {agent.id}):"
                        f"\n  Position: ({agent.position.position.x:.2f}, {agent.position.position.y:.2f})"
                        f"\n  Behavior Type: {agent.behavior.type}"
                        f"\n  Current State: {agent.behavior.state}"
                        f"\n  Linear Velocity: {agent.linear_vel:.2f}"
                        f"\n  Angular Velocity: {agent.angular_vel:.2f}"
                    )
        except Exception as e:
            self.node.get_logger().error(
                f"compute_agents service test failed: {str(e)}")

        # 2. Test compute_agent service
        try:
            self.node.get_logger().info("\n--- Testing /compute_agent service ---")
            request = ComputeAgent.Request()
            request.id = test_agent.id

            self.node.get_logger().info(
                f"Requesting compute_agent for ID: {request.id}")

            future = self._compute_agent_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            if future.result():
                response = future.result()
                agent = response.updated_agent
                self.node.get_logger().info(
                    f"\nCompute_agent response:"
                    f"\n  Agent: {agent.name} (ID: {agent.id})"
                    f"\n  Position: ({agent.position.position.x:.2f}, {agent.position.position.y:.2f})"
                    f"\n  Behavior: Type={agent.behavior.type}, State={agent.behavior.state}"
                )
        except Exception as e:
            self.node.get_logger().error(
                f"compute_agent service test failed: {str(e)}")

        # 3. Test move_agent service
        try:
            self.node.get_logger().info("\n--- Testing /move_agent service ---")
            request = MoveAgent.Request()
            request.agent_id = test_agent.id
            request.robot = test_robot
            request.current_agents = test_agents

            self.node.get_logger().info(
                f"Requesting move_agent for ID: {request.agent_id}")

            future = self._move_agent_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            if future.result():
                response = future.result()
                agent = response.updated_agent
                self.node.get_logger().info(
                    f"\nMove_agent response:"
                    f"\n  Agent: {agent.name} (ID: {agent.id})"
                    f"\n  New Position: ({agent.position.position.x:.2f}, {agent.position.position.y:.2f})"
                    f"\n  New Yaw: {agent.yaw:.2f}"
                    f"\n  Behavior State: {agent.behavior.state}"
                )
        except Exception as e:
            self.node.get_logger().error(
                f"move_agent service test failed: {str(e)}")

        self.node.get_logger().info("\n========= HUNAV SERVICES TEST COMPLETE =========")

    def setup_services(self):
        """Initialize all required services"""
        self.node.get_logger().info("Setting up Hunavservices...")

        # Debug namespace information
        self.node.get_logger().info(f"Node namespace: {self.node.get_namespace()}")
        self.node.get_logger().info(f"Task generator namespace: {self._namespace}")

        # Create service names with full namespace path (now using root namespace)
        service_names = {
            'compute_agent': f'/{self.SERVICE_COMPUTE_AGENT}',        # Added leading slash
            'compute_agents': f'/{self.SERVICE_COMPUTE_AGENTS}',      # Added leading slash
            'move_agent': f'/{self.SERVICE_MOVE_AGENT}',             # Added leading slash
            'reset_agents': f'/{self.SERVICE_RESET_AGENTS}'          # Added leading slash
        }

        # Log service names
        for service, full_name in service_names.items():
            self.node.get_logger().info(f"Creating service client for {service} at: {full_name}")

        # Create service clients
        self._compute_agent_client = self.node.create_client(
            ComputeAgent,
            service_names['compute_agent']
        )
        self._compute_agents_client = self.node.create_client(
            ComputeAgents,
            service_names['compute_agents']
        )
        self._move_agent_client = self.node.create_client(
            MoveAgent,
            service_names['move_agent']
        )
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
            while attempts < max_attempts:
                if client.wait_for_service(timeout_sec=2.0):
                    self.node.get_logger().info(f'Service {name} is available')
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
                return False

        self.node.get_logger().info("All services are ready")
        return True

    def create_pedestrian_sdf(self, agent_config: Dict) -> str:
        """Create SDF description for pedestrian (from WorldGenerator)"""
        skin_type = self.SKIN_TYPES.get(
            agent_config.get(
                'skin', 0), 'casual_man.dae')

        # Get path to mesh file
        mesh_path = os.path.join(
            get_package_share_directory('hunav_sim'),
            'hunav_rviz2_panel/meshes',
            skin_type
        )

        # Height adjustment based on skin type (from HuNavPlugin)
        height_adjustments = {'elegant_man.dae': 0.96,
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
        """Spawn dynamic obstacles/agents with enhanced debug output"""
        # # Initial debug prints
        # print("\n==================== STARTING SPAWN PROCESS ====================")
        # print(f"Attempting to spawn {len(list(obstacles))} obstacles")
        self.node.get_logger().error(
            f"Attempting to spawn {len(list(obstacles))} obstacles")

        for _obstacle in obstacles:

            obstacle = HunavDynamicObstacle.parse(
                _obstacle.extra, _obstacle.model)

            # print("\n=============== NEW OBSTACLE PROCESSING ===============")
            # print(f"Processing obstacle: {obstacle.name}")
            self.node.get_logger().error(
                f"Processing obstacle: {obstacle.name}")

            # Create Hunav Agent
            request = ComputeAgent.Request()
            agent = Agent()  # create an hunav_msgs/Agent

            try:
                # # Basic Properties Debug
                # print("\n--- Basic Properties Debug ---")
                # print(f"ID: {obstacle.id}")
                # print(f"Type: {obstacle.type}")
                # print(f"Skin: {obstacle.skin}")
                # print(f"Name: {obstacle.name}")
                # print(f"Group ID: {obstacle.group_id}")
                self.node.get_logger().error(
                    f"Basic Properties - ID: {obstacle.id}, Type: {obstacle.type}, Skin: {obstacle.skin}")

                # Set basic properties
                agent.id = obstacle.id
                agent.type = obstacle.type
                agent.skin = obstacle.skin
                agent.name = obstacle.name
                agent.group_id = obstacle.group_id
            except Exception as e:
                print(f"ERROR in basic properties: {e}")
                self.node.get_logger().error(f"ERROR in basic properties: {e}")

            try:
                # # Position Debug
                # print("\n--- Position & Orientation Debug ---")
                # print(f"Position object: {obstacle.position}")
                # print(f"Position type: {type(obstacle.position)}")
                # print(f"Yaw value: {obstacle.yaw}")
                self.node.get_logger().error(
                    f"Position Data: {obstacle.position}, Yaw: {obstacle.yaw}")

                # Set position
                agent.position = obstacle.position
                agent.yaw = obstacle.yaw
            except Exception as e:
                print(f"ERROR in position setting: {e}")
                self.node.get_logger().error(f"ERROR in position setting: {e}")

            try:
                # # Velocity Debug
                # print("\n--- Velocity Debug ---")
                # print(f"Velocity object: {obstacle.velocity}")
                # print(f"Desired velocity: {obstacle.desired_velocity}")
                # print(f"Linear vel: {obstacle.linear_vel}")
                # print(f"Angular vel: {obstacle.angular_vel}")
                # print(f"Radius: {obstacle.radius}")
                self.node.get_logger().error(
                    f"Velocity Data - Desired: {obstacle.desired_velocity}, Linear: {obstacle.linear_vel}")

                # Set velocity
                agent.velocity = obstacle.velocity if obstacle.velocity else Twist()
                agent.desired_velocity = obstacle.desired_velocity
                agent.radius = obstacle.radius
                agent.linear_vel = obstacle.linear_vel
                agent.angular_vel = obstacle.angular_vel
            except Exception as e:
                print(f"ERROR in velocity setting: {e}")
                self.node.get_logger().error(f"ERROR in velocity setting: {e}")

            try:
                # # Behavior Debug
                # print("\n--- Behavior Debug ---")
                # print(f"Behavior type: {obstacle.behavior.type}")
                # print(f"Configuration: {obstacle.behavior.configuration}")
                # print(f"Duration: {obstacle.behavior.duration}")
                # print("Force Factors:")
                # print(f"- Goal: {obstacle.behavior.goal_force_factor}")
                # print(f"- Obstacle: {obstacle.behavior.obstacle_force_factor}")
                # print(f"- Social: {obstacle.behavior.social_force_factor}")
                # print(f"- Other: {obstacle.behavior.other_force_factor}")
                # self.node.get_logger().error(
                # f"Behavior Data - Type: {obstacle.behavior.type}, Config:
                # {obstacle.behavior.configuration}")

                # Set behavior
                agent.behavior = AgentBehavior()
                agent.behavior.type = obstacle.behavior.type
                agent.behavior.configuration = obstacle.behavior.configuration
                agent.behavior.duration = obstacle.behavior.duration
                agent.behavior.once = obstacle.behavior.once
                agent.behavior.vel = obstacle.behavior.vel
                agent.behavior.dist = obstacle.behavior.dist
                agent.behavior.goal_force_factor = obstacle.behavior.goal_force_factor
                agent.behavior.obstacle_force_factor = obstacle.behavior.obstacle_force_factor
                agent.behavior.social_force_factor = obstacle.behavior.social_force_factor
                agent.behavior.other_force_factor = obstacle.behavior.other_force_factor
            except Exception as e:
                print(f"ERROR in behavior setting: {e}")
                self.node.get_logger().error(f"ERROR in behavior setting: {e}")

            try:
                # # Goals Debug
                # print("\n--- Goals Debug ---")
                # print(f"Number of goals: {len(obstacle.goals)}")
                # for i, goal in enumerate(obstacle.goals):
                #     print(f"Goal {i}: {goal}")
                # print(f"Cyclic goals: {obstacle.cyclic_goals}")
                # print(f"Goal radius: {obstacle.goal_radius}")
                # self.node.get_logger().error(
                # f"Goals Data - Count: {len(obstacle.goals)}, Cyclic:
                # {obstacle.cyclic_goals}")

                # Set goals
                agent.goals = obstacle.goals
                agent.cyclic_goals = obstacle.cyclic_goals
                agent.goal_radius = obstacle.goal_radius
            except Exception as e:
                print(f"ERROR in goals setting: {e}")
                self.node.get_logger().error(f"ERROR in goals setting: {e}")

            try:
                # # Closest obstacles Debug
                # print("\n--- Closest Obstacles Debug ---")
                # print(
                #     f"Number of closest obstacles: {len(obstacle.closest_obs)}")
                # self.node.get_logger().error(
                #     f"Closest obstacles count: {len(obstacle.closest_obs)}")

                agent.closest_obs = obstacle.closest_obs
            except Exception as e:
                print(f"ERROR in closest obstacles setting: {e}")
                self.node.get_logger().error(
                    f"ERROR in closest obstacles setting: {e}")

            try:
                # # SDF Model Creation Debug
                # print("\n--- SDF Model Creation ---")
                # print("Creating SDF model...")
                self.node.get_logger().error("Starting SDF model creation")

                sdf = self.create_pedestrian_sdf(obstacle)
                # print("SDF model created successfully")

                # Create model with SDF
                obstacle = dataclasses.replace(
                    obstacle,
                    model=Model(
                        description=sdf,
                        type=ModelType.SDF,
                        name=obstacle.name,
                        path='',
                    )
                )
            except Exception as e:
                print(f"ERROR in SDF model creation: {e}")
                self.node.get_logger().error(
                    f"ERROR in SDF model creation: {e}")

            try:
                # Spawn Entity Debug
                # print("\n--- Entity Spawning ---")
                # print(f"Attempting to spawn entity: {obstacle.name}")
                spawn_success = self._simulator.spawn_entity(obstacle)
                # print(f"Spawn {'successful' if spawn_success else 'failed'}")
                self.node.get_logger().error(
                    f"Spawn result for {obstacle.name}: {'success' if spawn_success else 'failed'}")
            except Exception as e:
                print(f"ERROR in entity spawning: {e}")
                self.node.get_logger().error(f"ERROR in entity spawning: {e}")

            try:
                # # HuNav Registration Debug
                # print("\n--- HuNav Registration ---")
                # print("Registering with HuNav...")
                request.agent = agent
                future = self._compute_agent_client.call(request)

                if future.result():
                    # print("Successfully registered with HuNav")
                    self.node.get_logger().error("Successfully registered with HuNav")
                else:
                    print("Failed to register with HuNav")
                    self.node.get_logger().error("Failed to register with HuNav")
            except Exception as e:
                print(f"ERROR in HuNav registration: {e}")
                self.node.get_logger().error(
                    f"ERROR in HuNav registration: {e}")

            # Final steps
            known = self._known_obstacles.create_or_get(
                name=obstacle.name,
                obstacle=obstacle,
                hunav_spawned=True,
                layer=ObstacleLayer.INUSE,
            )

        #     print("\n=============== OBSTACLE PROCESSING COMPLETE ===============")
        #     self.node.get_logger().error("OBSTACLE PROCESSING COMPLETE")

        # print("\n==================== SPAWN PROCESS COMPLETE ====================")
        # self.node.get_logger().error("SPAWN PROCESS COMPLETE")

    def _update_pedestrians(self):
        """Update pedestrians (from HuNavPlugin's OnUpdate)"""
        with self._lock:
            current_time = time.time()

            # Get updates from Hunav
            request = ComputeAgents.Request()
            future = self._compute_agents_client.call(request)

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
                        self._simulator.move_entity(
                            name=agent.id, position=pos)

                        # Update animation (like HuNavPlugin)
                        dt = current_time - ped_data['last_update']
                        animation_factor = self._get_animation_factor(
                            agent.behavior)
                        ped_data['animation_time'] += dt * animation_factor

                        # Update animation state if needed
                        if agent.behavior.state != ped_data.get(
                                'current_state'):
                            self._update_agent_animation(
                                agent.id, agent.behavior)
                            ped_data['current_state'] = agent.behavior.state

                        ped_data['last_update'] = current_time

    def _get_animation_factor(self, behavior: AgentBehavior) -> float:
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

    def _update_agent_animation(self, agent_id: str, behavior: AgentBehavior):
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
            self.node.get_logger().warn(
                f"No configuration found for agent {agent_id}")
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

    def remove_obstacles(self, purge=ObstacleLayer.WORLD):
        """Remove obstacles based on purge level"""
        if not self._is_paused:
            self._is_paused = True
            request = ResetAgents.Request()
            self._reset_agents_client.call(request)

        while self._semaphore_reset:
            time.sleep(0.1)

        actions: List[Callable] = []
        self._semaphore_reset = True

        try:
            to_forget: List[str] = list()

            for obstacle_id, obstacle in self._known_obstacles.items():
                if purge >= obstacle.layer:
                    def delete_entity(obstacle_id):
                        obstacle.hunav_spawned = False
                        self._simulator.delete_entity(name=obstacle_id)
                    actions.append(
                        functools.partial(
                            delete_entity, obstacle_id))
                    to_forget.append(obstacle_id)
                    obstacle.hunav_spawned = False

            for obstacle_id in to_forget:
                self._known_obstacles.forget(name=obstacle_id)
                if obstacle_id in self._pedestrians:
                    del self._pedestrians[obstacle_id]

        finally:
            self._semaphore_reset = False

        for action in actions:
            action()

    def move_robot(self, name: str, position: PositionOrientation):
        """Move robot to new position using HuNavSim's move_agent service"""
        try:
            if not self._move_agent_client.service_is_ready():
                self.node.get_logger().warn("Move agent service not ready yet")
                return

            # Create Robot Agent
            robot_agent = Agent()
            robot_agent.id = 0  # Robot typically uses ID 0
            robot_agent.name = name
            robot_agent.type = Agent.ROBOT
            robot_agent.position = position.to_pose()
            robot_agent.yaw = position.orientation

            # Create service request
            request = MoveAgent.Request()
            request.agent_id = 0
            request.robot = robot_agent
            request.current_agents = Agents()
            request.current_agents.header.stamp = self.node.get_clock().now().to_msg()
            request.current_agents.header.frame_id = "map"

            future = self._move_agent_client.call_async(request)

            timeout_sec = 1.0
            start_time = time.time()
            while not future.done() and time.time() - start_time < timeout_sec:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            if not future.done():
                self.node.get_logger().error("Move robot service call timed out")
                return

            if future.result() is not None:
                self.node.get_logger().info(f"Successfully moved robot {name}")
            else:
                self.node.get_logger().error("Move robot service call failed")

        except Exception as e:
            self.node.get_logger().error(f"Error moving robot: {str(e)}")

    def spawn_robot(self, robot: Robot):
        """Spawn robot in simulation"""
        self._simulator.spawn_entity(robot)

    def remove_robot(self, name: str):
        return self._simulator.delete_entity(name)

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi] (from HuNavPlugin)"""
        return math.fmod(angle, math.pi)

    def unuse_obstacles(self):
        """Remove/unuse all obstacles."""
        import traceback
        try:

            self.remove_obstacles(ObstacleLayer.WORLD)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error in unuse_obstacles: {e}")
            print(traceback.format_exc())
            return False
