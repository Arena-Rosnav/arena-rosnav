import re

class Matcher:
    GLOBAL_PLAN = lambda robot_name: re.compile(f"{robot_name}.*/global_plan$")
    LASER_SCAN = lambda robot_name: re.compile(f"{robot_name}/scan$")
    GLOBAL_COSTMAP = lambda robot_name: re.compile(f"{robot_name}.*/global_costmap/costmap$")
    LOCAL_COSTMAP = lambda robot_name: re.compile(f"{robot_name}.*/local_costmap/costmap$")
    GOAL = lambda robot_name: re.compile(f"{robot_name}/(?!.*move_base_flex/)goal$")
    MODEL = lambda robot_name: re.compile(f"{robot_name}/visualize/model")
