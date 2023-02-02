import traceback
import rospy
import roslaunch
import rospkg
import os
import yaml
import time
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

from task_generator.constants import Constants
from task_generator.utils import Utils


class RobotManager:
    """
        The robot manager manages the goal and start 
        position of a robot for all task modes.
    """

    def __init__(self, namespace, map_manager, environment, robot_setup):
        self.namespace = namespace
        self.namespace_prefix = "" if namespace == "" else "/" + namespace + "/"
        self.ns_prefix = lambda *topic: os.path.join(self.namespace, *topic)

        self.map_manager = map_manager
        self.environment = environment

        self.start_pos = [0, 0]
        self.goal_pos = [0, 0]

        self.goal_radius = rospy.get_param("goal_radius", 0.7) + 1
        self.is_goal_reached = False

        self.robot_setup = robot_setup
        self.record_data = rospy.get_param('record_data', False)#  and rospy.get_param('task_mode', 'scenario') == 'scenario'

        self.position = self.start_pos

    def set_up_robot(self):
        if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
            self.robot_radius = rospy.get_param("robot_radius")

        self.environment.spawn_robot(self.namespace, self.robot_setup["model"], self._robot_name())

        self.move_base_goal_pub = rospy.Publisher(self.ns_prefix(self.namespace, "move_base_simple", "goal"), PoseStamped, queue_size=10)
        self.pub_goal_timer = rospy.Timer(rospy.Duration(0.25), self.publish_goal_periodically)

        rospy.Subscriber(
            os.path.join(self.namespace, "odom"), 
            Odometry, 
            self.robot_pos_callback
        )

        if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
            return

        self.launch_robot(self.robot_setup)

        self.robot_radius = rospy.get_param(
            os.path.join(
                self.namespace, "robot_radius"
            )
        )

        # rospy.wait_for_service(os.path.join(self.namespace, "move_base", "clear_costmaps"))
        self._clear_costmaps_srv = rospy.ServiceProxy(
            self.ns_prefix(self.namespace, "move_base", "clear_costmaps"), 
            Empty
        )

    def _robot_name(self):
        if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
            return ""

        return self.namespace

    def reset(self, forbidden_zones=[], start_pos=None, goal_pos=None, move_robot=True):
        """
            The manager creates new start and goal position
            when a task is reset, publishes the goal to
            move base and rviz and moves the robot to
            the start position.
        """
        self.start_pos, self.goal_pos = self.generate_new_start_and_goal(
            forbidden_zones, start_pos, goal_pos
        )

        if self.record_data:
            rospy.set_param(os.path.join(self.namespace, "goal"), str(list(self.goal_pos)))
            rospy.set_param(os.path.join(self.namespace, "start"), str(list(self.start_pos)))

        self.publish_goal(self.goal_pos)

        if move_robot:
            self.move_robot_to_start()

        self.set_is_goal_reached(self.start_pos, self.goal_pos)

        time.sleep(0.1)

        try:
            self._clear_costmaps_srv()
        except:
            pass

        return self.position, self.goal_pos # self.start_pos, self.goal_pos

    def publish_goal_periodically(self, _):
        if self.goal_pos != None:
            self.publish_goal(self.goal_pos)

    def generate_new_start_and_goal(self, forbidden_zones, start_pos, goal_pos):
        new_start_pos = self._default_position(
            start_pos,
            self.map_manager.get_random_pos_on_map(
                self.robot_radius + Constants.RobotManager.SPAWN_ROBOT_SAFE_DIST,
                forbidden_zones
            )
        )

        new_goal_pos = self._default_position(
            goal_pos,
            self.map_manager.get_random_pos_on_map(
                self.robot_radius + Constants.RobotManager.SPAWN_ROBOT_SAFE_DIST,
                [
                    *forbidden_zones,
                    (
                        new_start_pos[0], 
                        new_start_pos[1], 
                        self.goal_radius
                    )
                ]
            )
        )

        return new_start_pos, new_goal_pos

    def publish_goal(self, goal):
        goal_msg = PoseStamped()
        goal_msg.header.seq = 0
        goal_msg.header.stamp = rospy.get_rostime()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal[0]
        goal_msg.pose.position.y = goal[1]

        goal_msg.pose.orientation.w = 0
        goal_msg.pose.orientation.x = 0
        goal_msg.pose.orientation.y = 0
        goal_msg.pose.orientation.z = 1

        self.move_base_goal_pub.publish(goal_msg)

    def move_robot_to_start(self):
        if not self.start_pos == None:
            self.move_robot_to_pos(self.start_pos)

    def move_robot_to_pos(self, pos):
        self.environment.move_robot(pos, name=self.namespace)

    def _default_position(self, pos, callback_pos):
        if not pos == None:
            return pos

        return callback_pos

    def launch_robot(self, robot_setup):
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
            ["arena_bringup", "robot.launch"]
        )

        print("START WITH MODEL", robot_setup["model"])

        args = [
            f"model:={robot_setup['model']}",
            f"local_planner:={robot_setup['planner']}",
            f"namespace:={self.namespace}",
            f"complexity:={rospy.get_param('complexity', 1)}",
            f"record_data:={self.record_data}",
            *([f"agent_name:={robot_setup.get('agent')}"] if robot_setup.get('agent') else [])
        ]

        self.process = roslaunch.parent.ROSLaunchParent(
            roslaunch.rlutil.get_or_generate_uuid(None, False),
            [(*roslaunch_file, args)]
        )
        self.process.start()

        # Overwrite default move base params
        base_frame = rospy.get_param(os.path.join(self.namespace, "robot_base_frame"))
        sensor_frame = rospy.get_param(os.path.join(self.namespace, "robot_sensor_frame"))
        rospy.set_param(
            os.path.join(self.namespace, "move_base", "global_costmap", "robot_base_frame"),
            self.namespace.replace("/", "") + "/" + base_frame
        )
        rospy.set_param(
            os.path.join(self.namespace, "move_base", "local_costmap", "robot_base_frame"),
            self.namespace.replace("/", "") + "/" + base_frame
        )
        rospy.set_param(
            os.path.join(self.namespace, "move_base", "local_costmap", "scan", "sensor_frame"),
            self.namespace.replace("/", "") + "/" + sensor_frame
        )
        rospy.set_param(
            os.path.join(self.namespace, "move_base", "global_costmap", "scan", "sensor_frame"),
            self.namespace.replace("/", "") + "/" + base_frame
        )

    def robot_pos_callback(self, data):
        current_position = data.pose.pose.position

        self.position = [current_position.x, current_position.y]

        self.set_is_goal_reached(
            self.position,
            self.goal_pos
        )

    def set_is_goal_reached(self, start, goal):
        distance_to_goal = math.sqrt(
            (start[0] - goal[0]) ** 2
            + (start[1] - goal[1]) ** 2 
        )

        self.is_goal_reached = distance_to_goal < self.goal_radius

    def is_done(self):
        return self.is_goal_reached