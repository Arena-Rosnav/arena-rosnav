import math
import os
import re
import time
import traceback
import typing

import attrs
import geometry_msgs.msg
from ament_index_python.packages import get_package_share_directory
from arena_rclpy_mixins.shared import Namespace
from geometry_msgs.msg import Point
from hunav_msgs.msg import Agent, AgentBehavior, Agents, WallSegment
from hunav_msgs.srv import (ComputeAgent, ComputeAgents, DeleteActors,
                            GetAgents, GetWalls, MoveAgent, ResetAgents)

from task_generator.constants import Constants
from task_generator.manager.entity_manager.dummy_manager import \
    DummyEntityManager
from task_generator.shared import (DynamicObstacle, Model, ModelType,
                                   ModelWrapper, Obstacle, Pose, Position)
from task_generator.simulators import BaseSimulator

from . import HunavDynamicObstacle


def _create_robot_message():
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


class _PedestrianHelper:

    _ANIMATION_MAP = {
        AgentBehavior.BEH_REGULAR: "07_01-walk.bvh",
        AgentBehavior.BEH_IMPASSIVE: "69_02_walk_forward.bvh",
        AgentBehavior.BEH_SURPRISED: "137_28-normal_wait.bvh",
        AgentBehavior.BEH_SCARED: "142_17-walk_scared.bvh",
        AgentBehavior.BEH_CURIOUS: "07_04-slow_walk.bvh",
        AgentBehavior.BEH_THREATENING: "17_01-walk_with_anger.bvh"
    }

    _SKIN_TYPES = {
        0: 'elegant_man.dae',
        1: 'casual_man.dae',
        2: 'elegant_woman.dae',
        3: 'regular_man.dae',
        4: 'worker_man.dae',
        5: 'walk.dae'
    }

    _HEIGHTS = {
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

    @classmethod
    def plugin_entity(cls, namespace: str) -> Obstacle:

        sdf_content = f"""<?xml version="1.0" ?>
            <sdf version="1.9">
                <model name="hunav_plugin">
                    <static>true</static>
                    <link name="empty">
                        <visual name="visual">
                            <geometry>
                                <box>
                                    <size>0.01 0.01 0.01</size>
                                </box>
                            </geometry>
                        </visual>
                    </link>
                    <plugin name="HuNavSystemPluginIGN" filename="libHuNavSystemPluginIGN.so">
                        <update_rate>1000.0</update_rate>
                        <namespace>{namespace}</namespace>
                        <!-- <robot_name>jackal</robot_name> -->
                        <use_gazebo_obs>true</use_gazebo_obs>
                        <global_frame_to_publish>map</global_frame_to_publish>
                        <use_navgoal_to_start>false</use_navgoal_to_start>
                        <navgoal_topic>goal_pose</navgoal_topic>
                        <ignore_models>
                            <model>ground_plane</model>
                            <model>sun</model>
                        </ignore_models>
                    </plugin>
                </model>
            </sdf>"""

        return Obstacle(
            name="hunav_plugin",
            pose=Pose(Position(x=0.0, y=0.0, z=-1.0)),
            model=ModelWrapper.Constant("hunav_plugin", {
                ModelType.SDF: Model(
                    type=ModelType.SDF,
                    name="hunav_plugin",
                    description=sdf_content,
                    path="",
                )
            })
        )

    @classmethod
    def create_sdf(cls, agent_config: HunavDynamicObstacle) -> str:
        """Create SDF description for pedestrian using gz-sim actor format"""

        # Get skin type
        skin_type = cls._SKIN_TYPES.get(agent_config.skin, 'casual_man.dae')

        # Animation mapping based on behavior
        animation_file = cls._ANIMATION_MAP.get(agent_config.behavior.type, "07_01-walk.bvh")
        animation_file = '../models/walk.dae'  # temp

        # Construct paths
        mesh_path = os.path.join(
            get_package_share_directory('hunav_rviz2_panel'),
            'meshes/models',
            skin_type
        )

        animation_path = os.path.join(
            get_package_share_directory('hunav_rviz2_panel'),
            'meshes/animations',
            animation_file
        )

        # # Temporärer Logger für Debug
        # import rclpy
        # if not rclpy.ok():
        #     rclpy.init()
        # temp_node = rclpy.create_node('temp_debug_node')
        # logger = temp_node.get_logger()

        #         # DEBUG
        # logger.warn(f"DEBUG: Looking for animation at: {animation_path}")
        # logger.warn(f"DEBUG: Animation exists: {os.path.exists(animation_path)}")

        # # Cleanup
        # temp_node.destroy_node()

        # Create the SDF
        sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.9">
            <actor name="{agent_config.name}">
                <pose>{agent_config.init_pose.x} {agent_config.init_pose.y} {cls._HEIGHTS.get(agent_config.skin, 1.0)} 0 0 {agent_config.yaw}</pose>

                <skin>
                    <filename>{mesh_path}</filename>
                    <scale>1.0</scale>
                </skin>

                <animation name="walking">
                    <filename>{animation_path}</filename>
                    <scale>1.0</scale>
                    <interpolate_x>true</interpolate_x>
                </animation>
            </actor>
        </sdf>"""
        return sdf


class HunavManager(DummyEntityManager):
    """HunavManager with debug logging for tracking execution flow"""

    _pedestrians: dict[int, dict]
    _agents_container: Agents
    _get_agents_container: Agents
    _wall_points: list[geometry_msgs.msg.Point]

    # Service Names
    SERVICE_COMPUTE_AGENT = 'compute_agent'
    SERVICE_COMPUTE_AGENTS = 'compute_agents'
    SERVICE_MOVE_AGENT = 'move_agent'
    SERVICE_RESET_AGENTS = 'reset_agents'
    SERVICE_GET_AGENTS = 'get_agents'
    SERVICE_GET_WALLS = 'get_walls'
    SERVICE_DELETE_ACTORS = 'delete_actors'

    def __init__(self, namespace: Namespace, simulator: BaseSimulator):
        """Initialize HunavManager with debug logging"""
        super().__init__(namespace=namespace, simulator=simulator)
        # Detect Simulator Type to decide between Plugin or move_entity callback
        self._logger.info(f"Detected simulator type: {self._simulator_type}")

        self._logger.info("=== HUNAVMANAGER INIT START ===")
        self._logger.debug("Parent class initialized")

        # Initialize collections
        self._logger.debug("Initializing collections...")
        self._pedestrians = {}
        self._wall_points = []
        self._agents_container = Agents()  # Container to hold all registered agents
        self._get_agents_container = Agents()  # Container specifically just to send the Agent attributes to Hunavsystemplugin
        self._agents_container.header.frame_id = "map"
        self._logger.debug("Collections initialized")

        # Setup services
        self._logger.debug("Setting up services...")
        setup_success = self._setup_services()
        if not setup_success:
            self._logger.error("Service setup failed!")
        else:
            self._logger.info("Services setup complete")

        # Wait to be sure that all services are ready
        self._logger.debug("Waiting for services to be ready...")
        time.sleep(2.0)
        self._logger.debug("Service wait complete")

        self._logger.info("=== HUNAVMANAGER INIT COMPLETE ===")

        self._gz_plugin_spawned: bool = False

    @property
    def _simulator_type(self) -> Constants.Simulator:
        """Detect which simulator is being used"""
        return self.node.conf.Arena.SIMULATOR.value

    def _setup_services(self):
        """Initialize all required services with debug logging"""
        self._logger.info("=== SETUP_SERVICES START ===")

        # Debug namespace information
        self._logger.debug(f"Node namespace: {self.node.get_namespace()}")
        self._logger.debug(f"Task generator namespace: {self._namespace}")

        # Create service names with full namespace path
        service_names = {
            'compute_agent': self.node.service_namespace(self.SERVICE_COMPUTE_AGENT),
            'compute_agents': self.node.service_namespace(self.SERVICE_COMPUTE_AGENTS),
            'move_agent': self.node.service_namespace(self.SERVICE_MOVE_AGENT),
            'reset_agents': self.node.service_namespace(self.SERVICE_RESET_AGENTS),
            'get_agents': self.node.service_namespace(self.SERVICE_GET_AGENTS),
            'get_walls': self.node.service_namespace(self.SERVICE_GET_WALLS),
            'delete_actors': self.node.service_namespace(self.SERVICE_DELETE_ACTORS)

        }

        # Log service creation attempts
        for service, full_name in service_names.items():
            self._logger.info(f"Creating service client for {service} at: {full_name}")

        # Create service clients
        self._logger.debug("Creating compute_agent client...")
        self._compute_agent_client = self.node.create_client(
            ComputeAgent,
            service_names['compute_agent']
        )

        self._logger.debug("Creating compute_agents client...")
        self._compute_agents_client = self.node.create_client(
            ComputeAgents,
            service_names['compute_agents'],
        )

        self._logger.debug("Creating move_agent client...")
        self._move_agent_client = self.node.create_client(
            MoveAgent,
            service_names['move_agent'],
        )

        self._logger.debug("Creating reset_agents client...")
        self._reset_agents_client = self.node.create_client(
            ResetAgents,
            service_names['reset_agents'],
        )

        self._logger.debug("Creating delete_actors client...")
        self._delete_actors_client = self.node.create_client(
            DeleteActors,
            service_names['delete_actors'],
        )
       # Create GetAgents service provider
        self._logger.debug(f"Creating get_agents service provider at: {service_names['get_agents']}")
        self._get_agents_server = self.node.create_service(
            GetAgents,
            service_names['get_agents'],
            self._get_agents_callback,
        )
        self._logger.debug("GetAgents service provider created")

        # Create GetWalls service provider
        self._logger.debug(f"Creating get_walls service provider at: {service_names['get_walls']}")
        self._get_walls_service = self.node.create_service(
            GetWalls,
            service_names['get_walls'],
            self._get_walls_callback,
        )
        self._logger.debug("GetWalls service provider created")

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
            self._logger.debug(f"Waiting for service {name}...")

            while attempts < max_attempts:
                if client.wait_for_service(timeout_sec=2.0):
                    self._logger.debug(f'Service {name} is available')
                    break
                attempts += 1
                self._logger.debug(
                    f'Waiting for service {name} (attempt {attempts}/{max_attempts})\n'
                    f'Looking for service at: {service_names[name]}'
                )

            if attempts >= max_attempts:
                self._logger.error(
                    f'Service {name} not available after {max_attempts} attempts\n'
                    f'Was looking for service at: {service_names[name]}'
                )
                self._logger.error("=== SETUP_SERVICES FAILED ===")
                return False

        self._logger.info("=== SETUP_SERVICES COMPLETE ===")
        return True

    def _get_agents_callback(self, request, response):
        """Handle get_agents service request - return UNMODIFIED agents"""
        try:
            self._logger.error("=== GET AGENTS CALLBACK ===")
            self._logger.error(f"Returning {len(self._get_agents_container.agents)} agents")

            # Update timestamp
            self._get_agents_container.header.stamp = self.node.get_clock().now().to_msg()
            self._get_agents_container.header.frame_id = "map"

            # Return the UNMODIFIED container
            response.agents = self._get_agents_container

            # Debug output
            # for agent in self._get_agents_container.agents:
            #     self._logger.error(f"Sending agent {agent.name}: desired_velocity={agent.desired_velocity}, goal_force_factor={agent.behavior.goal_force_factor}")

            return response

        except Exception as e:
            self._logger.error(f"Error in get_agents_callback: {e}")
            response.agents = Agents()  # Empty response on error
            return response

    def _get_walls_callback(self, request, response):
        """Service callback für Wall-Segments"""
        response.walls = self._wall_segments
        self._logger.info(f"Sent {len(self._wall_segments)} wall segments")
        return response

    def _move_entity_callback(self):
        """Pedestrian Move Entity Callback for non gazebo simulators"""
        # Nur updaten wenn Agents vorhanden sind
        if not self._agents_container.agents:
            return

        # Timestamp aktualisieren
        self._agents_container.header.stamp = self.node.get_clock().now().to_msg()

        # HuNav Service aufrufen
        request = ComputeAgents.Request()
        request.robot = _create_robot_message()
        request.current_agents = self._agents_container

        try:
            response = self._compute_agents_client.call(request)
            if response:
                # move_entity for each agent
                for updated_agent in response.updated_agents.agents:
                    pose = Pose.from_msg(updated_agent.position)
                    self._simulator.move_entity(updated_agent.name, pose)

                    for i, agent in enumerate(self._agents_container.agents):
                        if agent.id == updated_agent.id:
                            self._agents_container.agents[i] = updated_agent
                            break

        except Exception as e:
            self._logger.error(f"Failed to update agent positions: {e}")

    def spawn_dynamic_obstacles(self, obstacles: typing.Collection[DynamicObstacle]):
        """Override to handle batch registration after all spawns"""
        self._logger.debug(f"=== SPAWNING {len(obstacles)} DYNAMIC OBSTACLES ===")

        # Call parent method which handles individual spawns
        super().spawn_dynamic_obstacles(obstacles)

        # Now all obstacles have been spawned - register them with HuNav
        if self._agents_container.agents:
            self._logger.error(f"All spawns complete. Registering {len(self._agents_container.agents)} agents with HuNav")

            # Update timestamp
            self._agents_container.header.stamp = self.node.get_clock().now().to_msg()

            # Create request
            request = ComputeAgents.Request()
            request.robot = _create_robot_message()
            request.current_agents = self._agents_container

            # Call HuNav service
            response = self._compute_agents_client.call(request)

            if response:
                self._logger.error(f"Successfully registered {len(response.updated_agents.agents)} agents")

                # Update local agents with response data
                for updated_agent in response.updated_agents.agents:

                    for i, agent in enumerate(self._agents_container.agents):
                        if agent.id == updated_agent.id:
                            self._agents_container.agents[i] = updated_agent
                            break

                    # Update pedestrians dictionary if exists
                    if updated_agent.id in self._pedestrians:
                        self._pedestrians[updated_agent.id]['agent'] = updated_agent

                if self._simulator_type != Constants.Simulator.GAZEBO:
                    self._logger.debug("Non-Gazebo detected - starting movement timer")
                    self._update_timer = self.node.create_timer(
                        0.1,  # 10 Hz
                        self._move_entity_callback
                    )
            else:
                self._logger.error("Failed to register agents with HuNav")
        else:
            self._logger.debug("No agents to register")

    def _spawn_dynamic_obstacle_impl(self, obstacle: DynamicObstacle) -> DynamicObstacle | None:
        """Create agent but don't register with HuNav yet"""
        # self._logger.error(f"=== spawn_dynamic_obstacles_aufruf===")

        try:
            # Get unique ID
            try:
                agent_number = int(obstacle.name.split('_')[-1])
                unique_id = agent_number
            except (ValueError, IndexError):
                unique_id = len(self._agents_container.agents) + 1

            hunav_obstacle = HunavDynamicObstacle.from_dynamic_obstacle(obstacle)
            hunav_obstacle = attrs.evolve(hunav_obstacle, id=unique_id)

            self._logger.info(f"Preparing agent {hunav_obstacle.name} (ID: {unique_id})")

            # Create agent message
            agent_msg = Agent()
            agent_msg.id = unique_id
            agent_msg.name = hunav_obstacle.name
            agent_msg.type = hunav_obstacle.type
            agent_msg.skin = hunav_obstacle.skin
            agent_msg.group_id = hunav_obstacle.group_id
            agent_msg.desired_velocity = hunav_obstacle.desired_velocity
            # self._logger.error(f"=== spawn_dynamic_obstacles_desired_velocity: {agent_msg.desired_velocity}===")
            agent_msg.radius = hunav_obstacle.radius

            # Set position
            agent_msg.position = geometry_msgs.msg.Pose()
            agent_msg.position.position.x = hunav_obstacle.init_pose.x
            agent_msg.position.position.y = hunav_obstacle.init_pose.y
            agent_msg.position.position.z = 1.250000
            agent_msg.yaw = hunav_obstacle.yaw

            # Set behavior
            agent_msg.behavior = AgentBehavior()
            agent_msg.behavior.type = hunav_obstacle.behavior.type
            agent_msg.behavior.configuration = hunav_obstacle.behavior.configuration
            agent_msg.behavior.duration = hunav_obstacle.behavior.duration
            agent_msg.behavior.once = hunav_obstacle.behavior.once
            agent_msg.behavior.vel = hunav_obstacle.behavior.vel
            agent_msg.behavior.dist = hunav_obstacle.behavior.dist
            agent_msg.behavior.goal_force_factor = 20.0  # hunav_obstacle.behavior.goal_force_factor
            agent_msg.behavior.obstacle_force_factor = hunav_obstacle.behavior.obstacle_force_factor
            agent_msg.behavior.social_force_factor = hunav_obstacle.behavior.social_force_factor
            agent_msg.behavior.other_force_factor = hunav_obstacle.behavior.other_force_factor

            # Set goals
            agent_msg.goal_radius = hunav_obstacle.goal_radius
            agent_msg.cyclic_goals = hunav_obstacle.cyclic_goals
            if hunav_obstacle.goals:
                agent_msg.goals = hunav_obstacle.goals.as_poses()
            else:
                # Default goals if none exist
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

            # Add wall obstacles
            # self._logger.debug(f"Hunav Manager Wallpoints: {self._wall_points}")
            # agent_msg.closest_obs.extend(self._wall_points)
            # self._logger.debug(f"Hunav Manager Closest Obstacles: {agent_msg.closest_obs}")

            # After creating the agent message:
            self._logger.debug(f"""            ##Complete Debug for the set attributes
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

            # Add to container - NO ComputeAgents call here!
            self._get_agents_container.agents.append(agent_msg)
            self._agents_container.agents.append(agent_msg)
            # self._logger.error(f"spawn_dynamic_obstacle_agents_container {self._agents_container}")

            # Store in pedestrians dictionary
            self._pedestrians[agent_msg.id] = {
                'last_update': time.time(),
                'current_state': agent_msg.behavior.state,
                'agent': agent_msg,
                'animation_time': 0.0
            }

            self._logger.info(f"Added agent {agent_msg.name} to container. Total agents: {len(self._agents_container.agents)}")

            if self._simulator_type == Constants.Simulator.GAZEBO:
                # spawn plugin if not already spawned
                if not self._gz_plugin_spawned:
                    self._simulator.spawn_entity(_PedestrianHelper.plugin_entity(self.node.service_namespace()))
                    self._gz_plugin_spawned = True

                # Create SDF with plugin for Gazebo
                sdf = _PedestrianHelper.create_sdf(hunav_obstacle)
                new_obstacle = attrs.evolve(
                    obstacle,
                    model=obstacle.model.override(
                        ModelType.SDF,
                        lambda model: model.replace(description=sdf), noload=True)
                )
                self._logger.info(f"Created SDF and loaded System Plugin for: {agent_msg.name}")
                return new_obstacle
            else:
                # For other simulators: use simple model without plugin
                self._logger.info(f"Using simple spawning for simulator: {self._simulator_type}")
                return obstacle  # Return original obstacle without SDF modification

        except Exception as e:
            self._logger.error(f"Error preparing agent: {str(e)}")
            self._logger.error(traceback.format_exc())
            return None

    def _spawn_walls_impl(self, walls) -> bool:

        self._wall_segments = []

        for i, wall in enumerate(walls):
            dx = wall.End.x - wall.Start.x
            dy = wall.End.y - wall.Start.y
            length = math.sqrt(dx * dx + dy * dy)

            segment = WallSegment()
            segment.id = i
            segment.start = Point(x=wall.Start.x, y=wall.Start.y, z=0.0)
            segment.end = Point(x=wall.End.x, y=wall.End.y, z=0.0)
            segment.length = length
            segment.height = wall.height

            self._wall_segments.append(segment)

        self._logger.info(f"Cached {len(self._wall_segments)} wall segments")
        self._logger.info(f"Wallsegments{self._wall_segments} ")
        return True

    def _remove_obstacles_impl(self):
        """Remove all spawned pedestrians from simulation safely"""
        self._logger.info(f"=== REMOVING {len(self._pedestrians)} PEDESTRIANS ===")

        # Phase 1: Reset HuNav agents FIRST
        success = self._call_delete_actors_service()

        if not success:
            self._logger.error("Failed to delete  HuNav agents from ECM - continuing anyway")
            # Don't return False - continue with deletion

        # Phase 2: Clear local agents container
        self._agents_container = Agents()
        self._get_agents_container = Agents()
        self._logger.debug("Cleared local agents container")

        # Phase 3: Call plugin to delete actors from ECM
        success = self._reset_hunav_agents()
        if not success:
            self._logger.error("Failed to reset HuNav agents - continuing anyway")

        # Phase 4: Clean up local data
        self._clear_local_data()

        self._logger.info(f"Complete reset completed: {success}")
        return success

    def _reset_hunav_agents(self):
        """Reset HuNav by calling ResetAgents service"""
        try:
            if not self._reset_agents_client.wait_for_service(timeout_sec=2.0):
                self._logger.error("ResetAgents service not available")
                return False

            request = ResetAgents.Request()
            request.robot = _create_robot_message()

            # Empty agents = complete reset signal
            request.current_agents = Agents()
            request.current_agents.header.stamp = self.node.get_clock().now().to_msg()
            request.current_agents.header.frame_id = "map"

            self._logger.info("Calling HuNav ResetAgents service...")
            response = self._reset_agents_client.call(request)

            if response and response.ok:
                self._logger.info("HuNav reset successful - ready for new agents")
                return True
            else:
                self._logger.error("HuNav reset failed")
                return False

        except Exception as e:
            self._logger.error(f"Error calling ResetAgents: {e}")
            return False

    def _call_delete_actors_service(self):
        """Call the plugin's delete actors service"""
        try:
            if not self._delete_actors_client.wait_for_service(timeout_sec=2.0):
                self._logger.error("Delete actors service currently not available")
                return False

            request = DeleteActors.Request()

            self._logger.debug("Calling delete_actors service...")

            response = self._delete_actors_client.call(request)

            if response and response.success:
                self._logger.info(f"Successfully deleted {response.deleted_count} actors")
                return True
            else:
                self._logger.error("Delete actors service failed")
                return False

        except Exception as e:
            self._logger.error(f"Error calling delete_actors service: {e}")
            return False

    def _clear_local_data(self):
        """Clear all local data structures safely"""
        # Clear pedestrians dictionary
        self._pedestrians.clear()

        # Clear agents container (if not already done)
        self._agents_container.agents.clear()

        # Stop and cleanup movement timer if running (for non-gazebo simulators)
        if hasattr(self, '_update_timer') and self._update_timer:
            try:
                self._update_timer.destroy()
                self._update_timer = None
                self._logger.debug("Movement timer stopped and cleaned up")
            except Exception as e:
                self._logger.error(f"Error stopping movement timer: {e}")

        self._logger.debug("All local data structures cleared")
