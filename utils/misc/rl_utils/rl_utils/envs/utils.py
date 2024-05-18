import rospy

from task_generator.utils import Utils
from task_generator.constants import Constants


def sim_is_flatland():
    return Utils.get_simulator() == Constants.Simulator.FLATLAND

def sim_is_unity():
    return Utils.get_simulator() == Constants.Simulator.UNITY
