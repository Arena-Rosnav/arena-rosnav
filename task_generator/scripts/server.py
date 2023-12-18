#! /usr/bin/env python3


import os
from typing import Optional
import rospy

import dynamic_reconfigure.server
from task_generator.cfg import TaskGeneratorConfig


# class Callback:

#     PUBLISHERS: List[str]

#     _publishers: List[rospy.Publisher]

#     def __init__(self):
#         self._publishers = [rospy.Publisher(name, std_msgs.Empty) for name in self.PUBLISHERS]

#     def _publish(self):
#         for publisher in self._publishers:
#             publisher.publish(std_msgs.Empty())

#     def reconfigure(self, config):
#         ...

# class Callbacks:
#     registry : Dict[str, Callable] = dict()

#     @classmethod
#     def register(cls, name: str):
#         def wrapper(cb: Type[Callback]) -> Type[Callback]:
#             cls.registry[name] = cb().reconfigure
#             return cb
#         return wrapper

#     @classmethod
#     def get(cls, name: str):
#         return cls.registry.get(name, lambda *args, **kwargs: None)

# @Callbacks.register("SCENARIO")
# class Scenario(Callback):

#     PUBLISHERS = [
#         task_generator.tasks.obstacles.scenario.TM_Scenario.prefix(task_generator.tasks.obstacles.scenario.TM_Scenario.TOPIC_RECONFIGURE),
#         task_generator.tasks.robots.scenario.TM_Scenario.prefix(task_generator.tasks.robots.scenario.TM_Scenario.TOPIC_RECONFIGURE)
#     ]

#     def reconfigure(self, config):
#         print(config)
#         self._publish()

# levels: Dict[int, Callable] = {
#     int(level): Callbacks.get(name)
#     for (name, level)
#     in (
#         (
#             group.get("name"),
#             next(group.get("parameters")).get("level")
#         )
#         for group
#         in TaskGeneratorConfig.config_description.get("groups", {}))
# }

def run(namespace: Optional[str] = None):
    
    if namespace is None:
        namespace = str(rospy.get_name())
    
    def callback(config, level):
        return config

    server = dynamic_reconfigure.server.Server(
        type=TaskGeneratorConfig,
        callback=callback,
        namespace=namespace
    )
    
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("task_generator_server")
    run()
