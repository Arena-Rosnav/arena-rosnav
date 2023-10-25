from enum import Enum
from typing import Dict, List, Type
import pedsim_msgs.msg
import rospy


class Constants:
    TOPIC_SUBSCRIBE = "pedsim_waypoint_plugin/simulated_agents_input"
    TOPIC_PUBLISH = "pedsim_waypoint_plugin/simulated_agents_output"


class WaypointPluginName(Enum):
    PASSTHROUGH = "passthrough"
    SPINNY = "spinny"


AgentStates = List[pedsim_msgs.msg.AgentState]


class WaypointPlugin:
    def callback(self, agent_states: AgentStates) -> AgentStates:
        raise NotImplementedError()


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
            raise RuntimeError(f"plugin {plugin_name.value} not registered")

        plugin = plugin_class()

        rospy.loginfo(
            f"starting pedsim_waypoint_generator with plugin {type(plugin).__name__}")

        publisher = rospy.Publisher(
            name=Constants.TOPIC_PUBLISH,
            data_class=pedsim_msgs.msg.AgentStates,
            queue_size=1
        )

        def callback(msg: pedsim_msgs.msg.AgentStates):

            agent_states = msg.agent_states

            if agent_states is not None:
                msg = pedsim_msgs.msg.AgentStates()
                msg.agent_states = plugin.callback(agent_states)

            publisher.publish(msg)
            

        rospy.Subscriber(
            name=Constants.TOPIC_SUBSCRIBE,
            data_class=pedsim_msgs.msg.AgentStates,
            callback=callback,
            queue_size=1
        )
