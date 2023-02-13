import os
import rospkg
import subprocess
import yaml


class BaseSimulator:
    def __init__(self, namespace):
        self._namespace = namespace
        self._ns_prefix = lambda *topic: os.path.join(self._namespace, *topic)

    def before_reset_task(self):
        """
        Is executed each time before the task is reseted. This is useful in
        order to pause the simulation.
        """
        raise NotImplementedError()

    def after_reset_task(self):
        """
        Is executed after the task is reseted. This is useful to unpause the
        simulation.
        """
        raise NotImplementedError()

    def remove_all_obstacles(self):
        """
        Removes all obstacles from the current simulator. Does not remove
        the robot!
        """
        raise NotImplementedError()

    def spawn_random_dynamic_obstacle(self, **args):
        """
        Spawn a single random dynamic obstacle.
        
        Args:
            position: [int, int, int] denoting the x, y and angle.
            min_radius: minimal radius of the obstacle
            max_radius: maximal radius of the obstacle
            linear_vel: linear velocity
            angular_vel_max: maximal angular velocity
        """
        raise NotImplementedError()

    def spawn_random_static_obstacles(self, **args):
        """
        Spawn a single random static obstacle.
        
        Args:
            position: [int, int, int] denoting the x, y and angle.
            min_radius: minimal radius of the obstacle
            max_radius: maximal radius of the obstacle
        """
        raise NotImplementedError()

    def publish_goal(self, goal):
        """
        Publishes the goal. 
        """
        raise NotImplementedError()

    def move_robot(self, pos, name=None):
        """
        Move the robot to the given position. 
        """
        raise NotImplementedError()

    def spawn_robot(self, complexity=1):
        """
        Spawn a robot in the simulator.
        A position is not specified because the robot is moved at the 
        desired position anyway.
        """
        raise NotImplementedError()

    def spawn_pedsim_agents(self, agents):
        """
        
        """
        raise NotImplementedError()

    def reset_pedsim_agents(self):
        raise NotImplementedError()

    def spawn_obstacle(self, position, yaml_path=""):
        raise NotImplementedError()
    
    @staticmethod
    def get_robot_description(robot_name, namespace):
        arena_sim_path = rospkg.RosPack().get_path("arena-simulation-setup")
        
        return subprocess.check_output([
            "rosrun", 
            "xacro",
            "xacro",
            os.path.join(arena_sim_path, "robot", robot_name, "urdf", f"{robot_name}.urdf.xacro"),
            f"robot_namespace:={namespace}"
        ]).decode("utf-8")

    @staticmethod
    def read_robot_parameters(robot_name):
        robot_param_path = os.path.join(
            rospkg.RosPack().get_path("arena-simulation-setup"),
            "robot",
            robot_name,
            "model_params.yaml"
        )

        with open(robot_param_path, "r") as file:
            return yaml.safe_load(file)