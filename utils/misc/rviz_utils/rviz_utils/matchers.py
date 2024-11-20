import re

class Matcher:
    GLOBAL_PLAN = lambda robot_name: re.compile(f"{robot_name}.*/global_plan$")
    LASER_SCAN = lambda robot_name: re.compile(f"{robot_name}/scan$")
    GLOBAL_COSTMAP = lambda robot_name: re.compile(f"{robot_name}.*/global_costmap/costmap$")
    LOCAL_COSTMAP = lambda robot_name: re.compile(f"{robot_name}.*/local_costmap/costmap$")
    CURRENT_GOAL = lambda robot_name: re.compile(f"{robot_name}/(?!.*move_base_flex/)current_goal$")
    SUBGOAL = lambda robot_name: re.compile(f"{robot_name}/(?!.*move_base_flex/)current_subgoal$")
    MODEL = lambda robot_name: re.compile(f"{robot_name}/visualize/model")