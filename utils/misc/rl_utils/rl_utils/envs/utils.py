import rospy

from task_generator.utils import Utils
from task_generator.constants import Constants

from rl_utils.utils.observation_collector.observation_units.base_collector_unit import BaseCollectorUnit
from rl_utils.utils.observation_collector.observation_units.unity_collector_unit import UnityCollectorUnit
from rl_utils.utils.observation_collector.observation_units.rgbd_collector_unit import RgbdCollectorUnit
from rl_utils.utils.observation_collector.observation_units.globalplan_collector_unit import GlobalplanCollectorUnit
from rl_utils.utils.observation_collector.observation_units.semantic_ped_unit import SemanticAggregateUnit


def get_obs_structure():
    structure = []
    
    # default units
    structure.append(BaseCollectorUnit)
    structure.append(GlobalplanCollectorUnit)
    structure.append(SemanticAggregateUnit)

    train_mode = rospy.get_param("train_mode", True)
    if sim_is_unity() and train_mode:
        structure.append(UnityCollectorUnit)

    enable_rgbd = rospy.get_param("rgbd/enabled", False)
    if enable_rgbd:
        structure.append(RgbdCollectorUnit)
    
    return structure

def sim_is_flatland():
    return Utils.get_simulator() == Constants.Simulator.FLATLAND

def sim_is_unity():
    return Utils.get_simulator() == Constants.Simulator.UNITY
