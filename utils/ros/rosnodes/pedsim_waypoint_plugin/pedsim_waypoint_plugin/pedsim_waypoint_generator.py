import dataclasses
from enum import Enum
from typing import Dict, List, Optional, Type, TypeVar
import pedsim_msgs.msg
import std_msgs.msg
import rospy


class Constants:
    TOPIC_SUBSCRIBE = "pedsim_waypoint_plugin/input"
    TOPIC_PUBLISH = "pedsim_waypoint_plugin/output"


class WaypointPluginName(Enum):
    PASSTHROUGH = "passthrough"
    SPINNY = "spinny"
    PYSOCIAL1 = "pysocial1"
    PYSOCIAL2 = "pysocial2"
    DEEPSOCIALFORCE = "deepsocialforce"
    EVACUATION = "evacuation"


InputMsg = pedsim_msgs.msg.WaypointPluginDataframe


@dataclasses.dataclass
class InputData:
    header: std_msgs.msg.Header
    agents: List[pedsim_msgs.msg.AgentState]
    robots: List[pedsim_msgs.msg.RobotState]
    groups: List[pedsim_msgs.msg.AgentGroup]
    waypoints: List[pedsim_msgs.msg.Waypoint]
    line_obstacles: List[pedsim_msgs.msg.Wall] #TODO rename to walls
    obstacles: List[pedsim_msgs.msg.Obstacle]


OutputData = List[pedsim_msgs.msg.AgentFeedback]

OutputMsg = pedsim_msgs.msg.AgentFeedbacks


class WaypointPlugin:
    def callback(self, data: InputData) -> OutputData:
        raise NotImplementedError()


T = TypeVar("T")


def NList(l: Optional[List[T]]) -> List[T]:
    return [] if l is None else l


class PedsimWaypointGenerator:

    __registry: Dict[WaypointPluginName, Type[WaypointPlugin]] = dict()

    @classmethod
    def register(cls, name: WaypointPluginName):
        def inner(plugin: Type[WaypointPlugin]):
            if cls.__registry.get(name) is not None:
                raise NameError(f"plugin {name} is already registered")

            cls.__registry[name] = plugin
            return plugin
        return inner

    def __init__(self, plugin_name: WaypointPluginName):

        plugin_class = self.__registry.get(plugin_name)

        if plugin_class is None:
            raise RuntimeError(f"Plugin {plugin_name.value} has no registered implementation.\nImplemented plugins: {[name.value for name in self.__registry.keys()]}")

        publisher = rospy.Publisher(
            name=Constants.TOPIC_PUBLISH,
            data_class=OutputMsg,
            queue_size=1
        )


        while True:

            running: bool = False

            if rospy.get_param("/resetting", True) == True:
                rospy.wait_for_message("/reset_end", std_msgs.msg.Empty)

            try:
                plugin: WaypointPlugin = plugin_class()
            except Exception as e:
                rospy.signal_shutdown(f"Could not initialize plugin {plugin_name.value}. Aborting.")
                raise RuntimeError(f"Could not initialize plugin {plugin_name.value}. Aborting.") from e


            def callback(dataframe: InputMsg):

                if not running or dataframe.agent_states is None or len(dataframe.agent_states) == 0:
                    return

                dataframe_data = InputData(
                    header=dataframe.header,
                    agents=NList(dataframe.agent_states),
                    robots=NList(dataframe.robot_states),
                    groups=NList(dataframe.simulated_groups),
                    waypoints=NList(dataframe.simulated_waypoints),
                    line_obstacles=NList(dataframe.walls),
                    obstacles=NList(dataframe.obstacles)
                )

                agent_states_data = plugin.callback(dataframe_data)

                publisher.publish(
                    OutputMsg(
                        agents=agent_states_data
                    )
                )

            sub = rospy.Subscriber(
                name=Constants.TOPIC_SUBSCRIBE,
                data_class=InputMsg,
                callback=callback,
                queue_size=1
            )

            running = True

            rospy.loginfo(
                f"starting pedsim_waypoint_generator with plugin {type(plugin).__name__}")
            
            rospy.wait_for_message("/reset_start", std_msgs.msg.Empty)

            running = False
            sub.unregister()
            del plugin