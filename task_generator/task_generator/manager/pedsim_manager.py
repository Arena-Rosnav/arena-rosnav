import rospkg
import os

from abc import abstractmethod
from pedsim_msgs.msg import Ped
from geometry_msgs.msg import Point
from task_generator.constants import Pedsim


class PedsimManager():  
    @abstractmethod
    def create_pedsim_msg(agent):
        msg = Ped()

        msg.id = agent["id"]
        msg.pos = Point(agent["pos"][0], agent["pos"][1], 0)
        msg.type = agent["type"]
        msg.vmax = agent["vmax"]

        msg.yaml_file = os.path.join(
            rospkg.RosPack().get_path("arena-simulation-setup"),
            agent["yaml_file"]
        )

        msg.number_of_peds = 1

        msg.start_up_mode = Pedsim.START_UP_MODE
        msg.wait_time = Pedsim.WAIT_TIME
        msg.trigger_zone_radius = Pedsim.TRIGGER_ZONE_RADIUS
        msg.chatting_probability = Pedsim.CHATTING_PROBABILITY
        msg.tell_story_probability = Pedsim.TELL_STORY_PROBABILITY
        msg.group_talking_probability = Pedsim.GROUP_TALKING_PROBABILITY
        msg.talking_and_walking_probability = Pedsim.TALKING_AND_WALKING_PROBABILITY
        msg.requesting_service_probability = Pedsim.REQUESTING_SERVICE_PROBABILITY
        msg.requesting_guide_probability = Pedsim.REQUESTING_GUIDE_PROBABILITY
        msg.requesting_follower_probability = Pedsim.REQUESTING_FOLLOWER_PROBABILITY
        msg.max_talking_distance = Pedsim.MAX_TALKING_DISTANCE
        msg.max_servicing_radius = Pedsim.MAX_SERVICING_RADIUS
        msg.talking_base_time = Pedsim.TALKING_BASE_TIME
        msg.tell_story_base_time = Pedsim.TELL_STORY_BASE_TIME
        msg.group_talking_base_time = Pedsim.GROUP_TALKING_BASE_TIME
        msg.talking_and_walking_base_time = Pedsim.TALKING_AND_WALKING_BASE_TIME
        msg.receiving_service_base_time = Pedsim.RECEIVING_SERVICE_BASE_TIME
        msg.requesting_service_base_time = Pedsim.REQUESTING_SERVICE_BASE_TIME
        msg.force_factor_desired = Pedsim.FORCE_FACTOR_DESIRED
        msg.force_factor_obstacle = Pedsim.FORCE_FACTOR_OBSTACLE
        msg.force_factor_social = Pedsim.FORCE_FACTOR_SOCIAL
        msg.force_factor_robot = Pedsim.FORCE_FACTOR_ROBOT
        msg.waypoint_mode = Pedsim.WAYPOINT_MODE

        msg.waypoints = [Point(wp[0], wp[1], 0) for wp in agent["waypoints"]]

        return msg