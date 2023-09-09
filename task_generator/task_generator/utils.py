import rospy
import os


class Utils:
    def get_simulator():
        return rospy.get_param("simulator", "flatland").lower()
    
    def get_arena_type():
        return os.getenv("ARENA_TYPE", "training").lower()
