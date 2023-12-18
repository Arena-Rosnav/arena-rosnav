#! /usr/bin/env python3

import functools
import os
from typing import Any, Callable, List, Optional

import rospkg
import yaml

import rospy

import watchdog.observers
import watchdog.events

import dynamic_reconfigure.client
from task_generator.cfg import TaskGeneratorConfig


def observe(file: str, callback: watchdog.events.FileSystemEventHandler):
    observer = watchdog.observers.Observer()
    observer.schedule(
        path=file,
        event_handler=callback
    )
    observer.start()
    return observer


def safe_callback(fn: Callable):
    def wrapper(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except KeyboardInterrupt as e:
            raise e
        except:
            pass

    return wrapper


def recursive_get(obj: Any, property: List[str], fallback: Any = None) -> Any:
    if not len(property):
        return fallback if obj is None else obj
    try:
        return recursive_get(dict(obj).get(property[0]), property[1:])
    except:
        return fallback


def run(namespace: Optional[str] = None):

    if namespace is None:
        namespace = rospy.get_namespace()

    client = dynamic_reconfigure.client.Client(
        name = os.path.join(namespace, "task_generator_server")
    )

    FILE_TASK_CONFIG = os.path.join(
        rospkg.RosPack().get_path("arena_bringup"),
        "configs",
        "task_generator.yaml"
    )

    class TaskConfigHandler(watchdog.events.FileSystemEventHandler):

        def __init__(self) -> None:
            super().__init__()
            self.reconfigure()

        @staticmethod
        def reconfigure():
            with open(FILE_TASK_CONFIG) as f:
                content = dict(yaml.load(f, yaml.FullLoader))

            rospy.logwarn("SENSING CHANGE OF SCENARIO PARAMS")

            rospy.logwarn(str({
                    **{
                        # "no_of_episodes": content.get("no_of_episodes")
                    },
                    **{
                        name: recursive_get(content, name.split("_"))
                        for name in
                        (
                            str(param.get("name", ""))
                            for group in TaskGeneratorConfig.config_description.get("groups", {})
                            for param in group.get("parameters", [])
                        )
                    }
                }))
            
            #rospy.logwarn(str(TaskGeneratorConfig.config_description))

            client.update_configuration(
                {
                    **{
                        # "no_of_episodes": content.get("no_of_episodes")
                    },
                    **{
                        name: recursive_get(content, name.split("_"))
                        for name in
                        (
                            str(param.get("name", ""))
                            for group in TaskGeneratorConfig.config_description.get("groups", [])
                            for param in group.get("parameters", [])
                        )
                    }
                }
            )

        def on_modified(self, event):
            @safe_callback
            def callback():
                self.reconfigure()
            callback()

    observers = [
        observe(path, cb)
        for path, cb
        in [
            (FILE_TASK_CONFIG, TaskConfigHandler())
        ]
    ]

    try:
        while True:
            for observer in observers:
                observer.join(1)
    except KeyboardInterrupt as e:
        raise e
    finally:
        for observer in observers:
            observer.stop()
        for observer in observers:
            observer.join()


if __name__ == "__main__":
    rospy.init_node("task_generator_filewatcher")
    run()
