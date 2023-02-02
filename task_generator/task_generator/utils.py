import rospy
import os


class Utils:
    def get_environment():
        return rospy.get_param("environment", "flatland").lower()
    
    def get_arena_type():
        return os.getenv("ARENA_TYPE", "training").lower()
