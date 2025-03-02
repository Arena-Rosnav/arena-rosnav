from typing import TYPE_CHECKING
from rl_utils.cfg import RobotCfg, TaskCfg

if TYPE_CHECKING:
    from rosnav_rl import SimulationStateContainer


# states from configs are parsed as args
# other states are read from rosparam
def get_arena_states(
    goal_radius: float,
    max_steps: int,
    is_discrete: bool,
    safety_distance: float,
    robot_cfg: RobotCfg,
    task_modules_cfg: TaskCfg,
) -> "SimulationStateContainer":
    import rosnav_rl.states.simulation as simulation_states

    robot_state = simulation_states.RobotState(
        radius=robot_cfg.robot_description.robot_radius,
        safety_distance=safety_distance,
        action_state=simulation_states.ActionState(
            is_discrete=is_discrete,
            actions=(
                robot_cfg.robot_description.actions.discrete
                if is_discrete
                else robot_cfg.robot_description.actions.continuous.model_dump()
            ),
            is_holonomic=robot_cfg.robot_description.is_holonomic,
            velocity_state=simulation_states.VelocityState(
                min_linear_vel=-2.0,
                max_linear_vel=2.0,
                min_translational_vel=-2.0,
                max_translational_vel=2.0,
                min_angular_vel=-4.0,
                max_angular_vel=4.0,
            ),
        ),
        laser_state=simulation_states.LaserState(
            attach_full_range_laser=robot_cfg.attach_full_range_laser,
            laser_max_range=robot_cfg.robot_description.laser.range,
            laser_num_beams=robot_cfg.robot_description.laser.num_beams,
        ),
    )

    task_state = simulation_states.TaskState(
        goal_radius=goal_radius,  # TODO: check where this is set - dynamic_reconfigure?
        max_steps=max_steps,
        semantic_state=simulation_states.SemanticState(
            num_ped_types=5,
            ped_min_speed_x=-5.0,
            ped_max_speed_x=5.0,
            ped_min_speed_y=-5.0,
            ped_max_speed_y=5.0,
            social_state_num=99,
        ),
        task_modules=simulation_states.TaskModuleState(
            tm_robots=task_modules_cfg.tm_robots,
            tm_obstacles=task_modules_cfg.tm_obstacles,
            tm_modules=task_modules_cfg.tm_modules,
        ),
    )

    return simulation_states.SimulationStateContainer(
        robot=robot_state, task=task_state
    )
