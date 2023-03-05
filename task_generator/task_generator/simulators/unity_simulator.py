import rospy
import os

from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion

from .base_simulator import BaseSimulator
from .simulator_factory import SimulatorFactory
from ..constants import UnitySimulatorConstants

from unity_msgs.srv import SpawnRobot, SpawnRobotRequest, MoveModel, MoveModelRequest

@SimulatorFactory.register("unity")
class UnitySimulator(BaseSimulator):
    def __init__(self, namespace):

        print("STARTING UP")

        rospy.wait_for_service("/unity/spawn_robot")
        rospy.wait_for_service("/unity/move_model")

        self._spawn_robot_srv = rospy.ServiceProxy("/unity/spawn_robot", SpawnRobot)
        self._move_model_srv = rospy.ServiceProxy("/unity/move_model", MoveModel)

        self.map_manager = None
        pass

    def before_reset_task(self):
        """
        Is executed each time before the task is reseted. This is useful in
        order to pause the simulation.
        """
        pass

    def after_reset_task(self):
        """
        Is executed after the task is reseted. This is useful to unpause the
        simulation.
        """
        pass

    def remove_all_obstacles(self):
        """
        Removes all obstacles from the current simulator. Does not remove
        the robot!
        """
        pass

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
        pass

    def spawn_random_static_obstacle(self, **args):
        """
        Spawn a single random static obstacle.
        
        Args:
            position: [int, int, int] denoting the x, y and angle.
            min_radius: minimal radius of the obstacle
            max_radius: maximal radius of the obstacle
        """
        pass

    def publish_goal(self, goal):
        """
        Publishes the goal. 
        """
        pass

    def move_robot(self, pos, name=None):
        """
        Move the robot to the given position. 
        """

        request = MoveModelRequest()

        request.model_name = name

        pose = Pose()
        pose.position.x = pos[0]
        pose.position.z = pos[1]

        print("MOVE MODEL TO ", pos)
        
        pose.orientation = Quaternion(
            *quaternion_from_euler(0.0, pos[2], 0.0, axes="sxyz")
        )
        request.pose = pose
        request.reference_frame = "world"

        self._move_model_srv(request)

    def spawn_robot(self, name, robot_name, namespace_appendix=""):
        """
        Spawn a robot in the simulator.
        A position is not specified because the robot is moved at the 
        desired position anyway.
        """
        print("SPAWN ROBOT CALLED")

        robot_description = UnitySimulator.get_robot_description(robot_name, namespace_appendix)

        robot_urdf_file_path = os.path.join(
            os.environ[UnitySimulatorConstants.UNITY_ROS_NAVIGATION],
            f"{robot_name}.urdf"
        )

        robot_parameters = UnitySimulator.read_robot_parameters(robot_name)

        UnitySimulator.write_urdf_file_to_unity_dir(
            robot_urdf_file_path,
            robot_description
        )

        request = SpawnRobotRequest()

        request.model_name = name
        request.model_urdf_path = robot_urdf_file_path
        request.model_namespace = namespace_appendix
        request.reference_frame = "world"

        linear_range = UnitySimulator.get_robot_linear_range(
            robot_parameters["actions"]["continuous"]["linear_range"]
        )

        request.additional_data = [
            robot_parameters["laser"]["angle"]["min"],
            robot_parameters["laser"]["angle"]["max"],
            robot_parameters["laser"]["angle"]["increment"],
            robot_parameters["laser"]["range"],
            robot_parameters["laser"]["num_beams"],

            *linear_range,
            *robot_parameters["actions"]["continuous"]["angular_range"]
        ]

        self._spawn_robot_srv(request)

        UnitySimulator.delete_robot_file_in_unity_dir(robot_urdf_file_path)

    def spawn_pedsim_agents(self, agents):
        """
        
        """
        pass

    def reset_pedsim_agents(self):
        pass

    def spawn_obstacle(self, position, yaml_path=""):
        pass

    @staticmethod
    def get_robot_linear_range(linear_range):
        if isinstance(linear_range, dict):
            return [*linear_range["x"], *linear_range["y"]]

        return [*linear_range, 0, 0]

    @staticmethod
    def write_urdf_file_to_unity_dir(robot_file_path, robot_urdf):
        with open(robot_file_path, "w") as file:
            file.write(robot_urdf)
            file.close()

    @staticmethod
    def delete_robot_file_in_unity_dir(robot_file_path):
        try:
            os.remove(robot_file_path)
        except:
            rospy.logwarn("World file could not be deleted")