from enum import Enum
import rospkg

PLANNER_CONFIG_PATH = f"{rospkg.RosPack().get_path('arena_bringup')}/configs/training/drl_switch/planners.yaml"
PLANNER_PARAM_PATH = (
    lambda planner: rospkg.RosPack().get_path("arena_simulation_setup")
    + f"/configs/mbf/local/{planner}_local_planner_params.yaml"
)
ROBOT_SPECIFIC_PATH = (
    lambda model, planner: f"{rospkg.RosPack().get_path('arena_simulation_setup')}"
    + f"/entities/robots/{model}/configs/mbf/{planner}_local_planner_params.yaml"
)

DMRC_SERVER = "move_base_flex"
DMRC_SERVER_ACTION = "move_base_legacy_relay"


class MBF_COMPATIBLE_TYPE:
    class INTER(Enum):
        pass

    class LOCAL(Enum):
        teb = "TebLocalPlannerROS"
        dwa = "DwaLocalPlannerROS"
        mpc = "MpcLocalPlannerROS"
