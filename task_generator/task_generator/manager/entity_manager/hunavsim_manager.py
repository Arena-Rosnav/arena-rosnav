import dataclasses
import functools
import time
from typing import Callable, List, Collection

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from hunav_msgs.srv import AddPedestrian, DeletePedestrian, UpdatePedestrianGoal
from hunav_msgs.msg import PedestrianState
from std_srvs.srv import Empty, Trigger

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
    SERVICE_ADD_PEDESTRIAN = 'hunav/add_pedestrian'
    SERVICE_DELETE_PEDESTRIAN = 'hunav/delete_pedestrian'
    SERVICE_UPDATE_GOAL = 'hunav/update_pedestrian_goal'
    SERVICE_PAUSE_SIMULATION = 'hunav/pause_simulation'
    SERVICE_RESUME_SIMULATION = 'hunav/resume_simulation'
    
    # Topics
    TOPIC_PEDESTRIAN_STATES = 'hunav/pedestrian_states'
    
    def __init__(self, namespace: Namespace, simulator: GazeboSimulator):
        """Initialize HunavsimManager
        
        Args:
            namespace (Namespace): Global namespace
            simulator (GazeboSimulator): Instance of GazeboSimulator
        """
        super().__init__(namespace=namespace, simulator=simulator)

        self._known_obstacles = KnownObstacles()
        
        # Initialize service clients
        self._add_pedestrian_client = self.create_client(
            AddPedestrian, 
            self._namespace(self.SERVICE_ADD_PEDESTRIAN)
        )
        self._delete_pedestrian_client = self.create_client(
            DeletePedestrian,
            self._namespace(self.SERVICE_DELETE_PEDESTRIAN)
        )
        self._update_goal_client = self.create_client(
            UpdatePedestrianGoal,
            self._namespace(self.SERVICE_UPDATE_GOAL)
        )
        self._pause_simulation_client = self.create_client(
            Empty,
            self._namespace(self.SERVICE_PAUSE_SIMULATION)
        )
        self._resume_simulation_client = self.create_client(
            Empty,
            self._namespace(self.SERVICE_RESUME_SIMULATION)
        )
        
        # Subscribe to pedestrian states
        self.create_subscription(
            PedestrianState,
            self._namespace(self.TOPIC_PEDESTRIAN_STATES),
            self._pedestrian_state_callback,
            10
        )

        # Initialize state variables
        self._is_paused = False
        self._semaphore_reset = False

        # Initialize JAIL_POS generator for handling deleted actors
        def gen_JAIL_POS(steps: int, x: int = 1, y: int = 0):
            steps = max(steps, 1)
            while True:
                x += y == steps
                y %= steps
                yield PositionOrientation(-x, y, 0)
                y += 1
        self.JAIL_POS = gen_JAIL_POS(10)


        
    @staticmethod
    def convert_pose(pose: Pose) -> PositionOrientation:
        return PositionOrientation(
            pose.position.x,
            pose.position.y,
            euler_from_quaternion(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
            )[2],
        )

    @staticmethod
    def pos_to_pose(pos: PositionOrientation) -> Pose:
        return Pose(
            position=Point(x=pos.x, y=pos.y, z=0),
            orientation=Quaternion(
                *quaternion_from_euler(0.0, 0.0, pos.orientation, axes="sxyz")
            ),
        )

    def spawn_dynamic_obstacles(self, obstacles: Collection[DynamicObstacle]):
        """Spawn pedestrians using Hunavsim"""
        for obstacle in obstacles:
            request = AddPedestrian.Request()
            request.id = obstacle.name
            request.pose = self.pos_to_pose(obstacle.position)
            
            # Add waypoints if available
            if obstacle.waypoints:
                request.goal = self.pos_to_pose(obstacle.waypoints[0])
            
            # Process model for Gazebo/Ignition
            obstacle = dataclasses.replace(
                obstacle,
                model=obstacle.model.override(
                    model_type=ModelType.SDF,
                    override=lambda model: model.replace(
                        description=SDFUtil.serialize(
                            SDFUtil.update_plugins(
                                namespace=self._simulator._namespace(str(request.id)),
                                description=SDFUtil.parse(model.description),
                            )
                        )
                    ),
                    name=request.id,
                ),
            )
            
            self._add_pedestrian_client.call_async(request)
            
            known = self._known_obstacles.create_or_get(
                name=obstacle.name,
                obstacle=obstacle,
                pedsim_spawned=False,
                layer=ObstacleLayer.INUSE,
            )

    def spawn_obstacles(self, obstacles):
        """Spawn static obstacles"""
        for obstacle in obstacles:
            known = self._known_obstacles.get(obstacle.name)
            if known is not None:
                if known.obstacle.name != obstacle.name:
                    raise RuntimeError(
                        f"new model name {obstacle.name} does not match model name {known.obstacle.name} of known obstacle {obstacle.name}"
                    )
                known.layer = ObstacleLayer.INUSE
            else:
                known = self._known_obstacles.create_or_get(
                    name=obstacle.name,
                    obstacle=obstacle,
                    pedsim_spawned=False,
                    layer=ObstacleLayer.INUSE,
                )
            
            # Spawn in simulator
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
            pedsim_spawned=False,
        )
        
        # Spawn walls in simulator
        self._simulator.spawn_entity(obstacle)

    def unuse_obstacles(self):
        """Mark all obstacles as unused"""
        self._is_paused = True
        self._pause_simulation_client.call_async(Empty.Request())

        for obstacle_id, obstacle in self._known_obstacles.items():
            if obstacle.layer == ObstacleLayer.INUSE:
                obstacle.layer = ObstacleLayer.UNUSED

    def remove_obstacles(self, purge):
        """Remove obstacles based on purge level"""
        if not self._is_paused:
            self._is_paused = True
            self._pause_simulation_client.call_async(Empty.Request())

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
                            # Move to JAIL instead of deletion for actors
                            def move_to_jail(obstacle_id):
                                jail = next(self.JAIL_POS)
                                self._simulator.move_entity(
                                    name=obstacle_id, position=jail
                                )
                            actions.append(functools.partial(move_to_jail, obstacle_id))
                        else:
                            def delete_entity(obstacle_id):
                                obstacle.pedsim_spawned = False
                                self._simulator.delete_entity(name=obstacle_id)
                            actions.append(functools.partial(delete_entity, obstacle_id))
                            to_forget.append(obstacle_id)
                    else:
                        obstacle.pedsim_spawned = False
                        to_forget.append(obstacle_id)

                    # Delete from Hunavsim
                    request = DeletePedestrian.Request()
                    request.id = obstacle_id
                    self._delete_pedestrian_client.call_async(request)

            for obstacle_id in to_forget:
                self._known_obstacles.forget(name=obstacle_id)

        finally:
            self._semaphore_reset = False

        for action in actions:
            action()

        self._resume_simulation_client.call_async(Empty.Request())
        self._is_paused = False

    def spawn_robot(self, robot: Robot):
        """Spawn robot in simulation"""
        self._simulator.spawn_entity(robot)

    def move_robot(self, name: str, position: PositionOrientation):
        """Move robot to new position"""
        self._simulator.move_entity(name=name, position=position)

    def _pedestrian_state_callback(self, msg: PedestrianState):
        """Handle updates about pedestrian positions"""
        if msg.id in self._known_obstacles:
            entity = self._known_obstacles.get(msg.id)
            
            if entity is None:
                return
                
            if entity.pedsim_spawned:
                # Update position in simulator
                self._simulator.move_entity(
                    name=msg.id,
                    position=self.convert_pose(msg.pose)
                )
            else:
                # First time seeing this pedestrian, spawn it in simulator
                self._simulator.spawn_entity(
                    Obstacle(
                        name=msg.id,
                        position=self.convert_pose(msg.pose),
                        model=entity.obstacle.model,
                        extra=entity.obstacle.extra,
                    )
                )
                entity.pedsim_spawned = True