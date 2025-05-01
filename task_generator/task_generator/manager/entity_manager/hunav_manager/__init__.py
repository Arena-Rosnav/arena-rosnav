
import enum
import os
import typing

import attrs
import geometry_msgs.msg
import yaml
from ament_index_python.packages import get_package_share_directory

from task_generator.shared import DynamicObstacle, ModelWrapper


@attrs.frozen()
class Position:
    x: float = attrs.field(converter=float)
    y: float = attrs.field(converter=float)
    z: float = attrs.field(default=0., converter=float)


@attrs.frozen()
class PositionH(Position):
    h: float = attrs.field(default=0., converter=float)


class Goals(list[Position]):
    @classmethod
    def parse(cls, obj: dict) -> "Goals":
        waypoints = [
            Position(
                x=waypoint.get('x', 0.),
                y=waypoint.get('y', 0.),
                z=waypoint.get('z', 0.),
            )
            for waypoint in (obj.get(wpname) for wpname in obj['goals'])
            if waypoint is not None
        ]
        return cls(waypoints)

    def as_poses(self) -> list[geometry_msgs.msg.Pose]:
        return [
            geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(
                    x=p.x,
                    y=p.y,
                    z=p.z
                )
            )
            for p
            in self
        ]


@attrs.frozen()
class HunavDynamicObstacle:

    @attrs.frozen()
    class Behavior:
        type: int
        state: int
        configuration: int
        duration: float
        once: bool
        vel: float
        dist: float
        social_force_factor: float
        goal_force_factor: float
        obstacle_force_factor: float
        other_force_factor: float

        _default: typing.ClassVar["HunavDynamicObstacle.Behavior"]

        @classmethod
        def parse(cls, obj: dict) -> "HunavDynamicObstacle.Behavior":
            return cls(
                type=obj.get('type', cls._default.type),
                state=obj.get('state', cls._default.state),
                configuration=obj.get(
                    'configuration', cls._default.configuration),
                duration=obj.get('duration', cls._default.duration),
                once=obj.get('once', cls._default.once),
                vel=obj.get('vel', cls._default.vel),
                dist=obj.get('dist', cls._default.dist),
                social_force_factor=obj.get(
                    'social_force_factor',
                    cls._default.social_force_factor),
                goal_force_factor=obj.get(
                    'goal_force_factor', cls._default.goal_force_factor),
                obstacle_force_factor=obj.get(
                    'obstacle_force_factor',
                    cls._default.obstacle_force_factor),
                other_force_factor=obj.get(
                    'other_force_factor', cls._default.other_force_factor),
            )

    id: int
    type: int = attrs.field(converter=lambda v: v if isinstance(v, int) else 1)
    skin: int
    name: str
    group_id: int
    init_pose: PositionH
    yaw: float
    model: ModelWrapper
    goals: Goals
    extra: dict
    velocity: None
    desired_velocity: float
    radius: float
    linear_vel: float
    angular_vel: float
    behavior: Behavior
    cyclic_goals: bool
    goal_radius: float
    closest_obs: list

    _default: typing.ClassVar["HunavDynamicObstacle"]

    @classmethod
    def from_dynamic_obstacle(cls, obj: DynamicObstacle, extra: dict | None = None) -> "HunavDynamicObstacle":
        if extra is None:
            extra = {}
        extra = {**obj.extra, **extra}

        if 'goals' in extra:
            waypoints = Goals.parse(extra)
        else:
            waypoints = Goals([
                Position(
                    x=waypoint.x,
                    y=waypoint.y,
                )
                for waypoint
                in obj.waypoints
            ])

        if 'behavior' in extra:
            behavior = cls.Behavior.parse(extra['behavior'])
        else:
            behavior = cls.Behavior._default

        return cls(
            name=obj.name,
            init_pose=PositionH(
                x=extra.get('position', {}).get('x', obj.position.x),
                y=extra.get('position', {}).get('y', obj.position.y),
                z=extra.get('position', {}).get('z', cls._default.init_pose.z),
                h=extra.get('position', {}).get('h', cls._default.init_pose.h),
            ),
            yaw=extra.get('yaw', obj.position.orientation),
            model=obj.model,
            goals=waypoints,
            velocity=extra.get('velocity', cls._default.velocity),
            desired_velocity=extra.get('desired_velocity', cls._default.desired_velocity),
            radius=extra.get('radius', cls._default.radius),
            linear_vel=extra.get('linear_vel', cls._default.linear_vel),
            angular_vel=extra.get('angular_vel', cls._default.angular_vel),
            behavior=behavior,
            cyclic_goals=extra.get('cyclic_goals', cls._default.cyclic_goals),
            goal_radius=extra.get('goal_radius', cls._default.goal_radius),
            closest_obs=[],
            extra=extra,
            id=extra.get('id', cls._default.id),
            type=extra.get('type', cls._default.type),
            skin=extra.get('skin', cls._default.skin),
            group_id=extra.get('group_id', cls._default.group_id),
        )

    @classmethod
    def parse(cls, obj: dict, model: ModelWrapper) -> "HunavDynamicObstacle":

        if 'goals' in obj:
            waypoints = Goals.parse(obj)
        else:
            waypoints = cls._default.goals

        return cls(
            name=obj.get('name', cls._default.name),
            extra=obj.get('extra', cls._default.extra),
            model=model,
            goals=waypoints,
            init_pose=PositionH(
                x=obj.get('init_pose', {}).get('x', cls._default.init_pose.x),
                y=obj.get('init_pose', {}).get('y', cls._default.init_pose.y),
                z=obj.get('init_pose', {}).get('z', cls._default.init_pose.z),
                h=obj.get('init_pose', {}).get('h', cls._default.init_pose.h),
            ),
            yaw=obj.get('yaw', cls._default.yaw),
            id=obj.get("id", cls._default.id),
            behavior=cls.Behavior.parse(obj.get('behavior', {})),
            type=obj.get('type', cls._default.type),
            skin=obj.get('skin', cls._default.skin),
            group_id=obj.get('group_id', cls._default.group_id),
            velocity=None,
            desired_velocity=obj.get('max_vel', cls._default.desired_velocity),
            radius=obj.get('radius', cls._default.radius),
            linear_vel=cls._default.linear_vel,
            angular_vel=cls._default.angular_vel,
            cyclic_goals=obj.get('cyclic_goals', cls._default.cyclic_goals),
            goal_radius=obj.get('goal_radius', cls._default.goal_radius),
            closest_obs=[],
        )


def _load_config(filename: str = "default.yaml") -> "HunavDynamicObstacle":
    """Load config from YAML file in arena_bringup configs."""

    # second priority: Install space
    config_path = os.path.join(
        get_package_share_directory("arena_bringup"),
        "configs",
        "hunav_agents",
        filename
    )

    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        agent_config = config['hunav_loader']['ros__parameters']['agent1']
        return HunavDynamicObstacle.parse(agent_config, ModelWrapper(''))

    except Exception as e:
        raise RuntimeError(f"Error loading config from {config_path}") from e


HunavDynamicObstacle.Behavior._default = HunavDynamicObstacle.Behavior(
    type=0,
    state=0,
    configuration=0,
    duration=0,
    once=False,
    vel=0.,
    dist=0.,
    social_force_factor=0.,
    goal_force_factor=0.,
    obstacle_force_factor=0.,
    other_force_factor=0.,


)

HunavDynamicObstacle._default = HunavDynamicObstacle(
    init_pose=PositionH(x=0, y=0),
    name='',
    model=ModelWrapper.EMPTY(),
    extra={},
    goals=Goals(),
    id=0,
    type=1,
    skin=0,
    group_id=0,
    yaw=0.,
    velocity=None,
    desired_velocity=0.,
    radius=0.,
    linear_vel=0.,
    angular_vel=0.,
    behavior=HunavDynamicObstacle.Behavior._default,
    cyclic_goals=False,
    goal_radius=0.,
    closest_obs=[],
)


HunavDynamicObstacle._default = _load_config()
# print("Loaded HunavDynamicObstacle:", HunavDynamicObstacle._default)
HunavDynamicObstacle.Behavior._default = HunavDynamicObstacle._default.behavior
# print("Behavior:", HunavDynamicObstacle.Behavior._default)


def test_hunav_services(self):
    """Test all HuNav services with debug logging"""
    self.node.get_logger().warn("=== TEST_HUNAV_SERVICES START ===")

    # Create test agents
    self.node.get_logger().warn("Creating test agents...")
    test_agents = Agents()
    test_agents.header.stamp = self.node.get_clock().now().to_msg()
    test_agents.header.frame_id = "map"

    # Create test agent
    self.node.get_logger().warn("Creating test pedestrian...")
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
    self.node.get_logger().warn("Setting test agent behavior...")
    test_agent.behavior = AgentBehavior()
    test_agent.behavior.type = AgentBehavior.BEH_REGULAR
    test_agent.behavior.configuration = AgentBehavior.BEH_CONF_DEFAULT
    test_agent.behavior.duration = 40.0
    test_agent.behavior.once = True
    test_agent.behavior.goal_force_factor = 2.0
    test_agent.behavior.obstacle_force_factor = 10.0
    test_agent.behavior.social_force_factor = 5.0

    # Add test goal
    self.node.get_logger().warn("Adding test goal...")
    goal = Pose()
    goal.position.x = 5.0
    goal.position.y = 5.0
    test_agent.goals.append(goal)
    test_agent.cyclic_goals = True
    test_agent.goal_radius = 0.3

    test_agents.agents.append(test_agent)

    # Create test robot
    self.node.get_logger().warn("Creating test robot...")
    test_robot = Agent()
    test_robot.id = 0
    test_robot.name = "test_robot"
    test_robot.type = Agent.ROBOT
    test_robot.position.position.x = 0.0
    test_robot.position.position.y = 0.0
    test_robot.yaw = 0.0
    test_robot.radius = 0.3

    try:
        self.node.get_logger().warn("Testing compute_agents service...")
        request = ComputeAgents.Request()
        request.robot = test_robot
        request.current_agents = test_agents

        self.node.get_logger().warn(f"Sending request with {len(request.current_agents.agents)} agents")
        # Change this line:
        response = self._compute_agents_client.call(request)  # Use synchronous call

        if response:
            self.node.get_logger().warn(f"Received response with {len(response.updated_agents.agents)} agents")
            for agent in response.updated_agents.agents:
                self.node.get_logger().warn(
                    f"\nAgent {agent.name} (ID: {agent.id}):"
                    f"\n  Position: ({agent.position.position.x:.2f}, {agent.position.position.y:.2f})"
                    f"\n  Behavior Type: {agent.behavior.type}"
                    f"\n  Current State: {agent.behavior.state}"
                    f"\n  Linear Velocity: {agent.linear_vel:.2f}"
                    f"\n  Angular Velocity: {agent.angular_vel:.2f}"
                )
    except Exception as e:
        self.node.get_logger().error(f"compute_agents service test failed: {str(e)}")

    # Test move_agent service
    try:
        self.node.get_logger().warn("Testing move_agent service...")
        request = MoveAgent.Request()
        request.agent_id = test_agent.id
        request.robot = test_robot
        request.current_agents = test_agents

        future = self._move_agent_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result():
            response = future.result()
            agent = response.updated_agent
            self.node.get_logger().warn(
                f"Move_agent response:"
                f"\n  Agent: {agent.name} (ID: {agent.id})"
                f"\n  New Position: ({agent.position.position.x:.2f}, {agent.position.position.y:.2f})"
                f"\n  New Yaw: {agent.yaw:.2f}"
                f"\n  Behavior State: {agent.behavior.state}"
            )
    except Exception as e:
        self.node.get_logger().error(f"move_agent service test failed: {str(e)}")

    self.node.get_logger().warn("=== TEST_HUNAV_SERVICES COMPLETE ===")


# Animation configuration (from WorldGenerator)
SKIN_TYPES: dict[int, str] = {
    0: 'elegant_man.dae',
    1: 'casual_man.dae',
    2: 'elegant_woman.dae',
    3: 'regular_man.dae',
    4: 'worker_man.dae',
    5: 'walk.dae'
}


class ANIMATION_TYPES(str, enum.Enum):
    WALK = '07_01-walk.bvh',
    WALK_FORWARD = '69_02_walk_forward.bvh',
    NORMAL_WAIT = '137_28-normal_wait.bvh',
    WALK_CHILDISH = '142_01-walk_childist.bvh',
    SLOW_WALK = '07_04-slow_walk.bvh',
    WALK_SCARED = '142_17-walk_scared.bvh',
    WALK_ANGRY = '17_01-walk_with_anger.bvh'
