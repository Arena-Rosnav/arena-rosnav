#! /usr/bin/env python3

import functools
import json
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
    
def encode(var: Any):
    if isinstance(var, list):
        return ";".join(var)
    if isinstance(var, dict):
        return json.dumps(var)
    return var

def get_or_ignore(obj: dict, key: str) -> dict:
    return {key: obj.get(key)} if key in obj else {}

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

            rospy.logdebug("SENSING CHANGE OF TASK_MODE PARAMS")

            client.update_configuration(
                {
                    **{
                        k:v
                        for d in [get_or_ignore(content, parameter["name"]) for parameter in TaskGeneratorConfig.config_description.get("parameters", [])]
                        for k,v in d.items()
                    },
                    **{
                        k:v
                        for (k,v)
                        in (
                            (name, encode(recursive_get(content, name.split("_"))))
                            for name in
                            (
                                str(parameter["name"])
                                for group in TaskGeneratorConfig.config_description.get("groups", [])
                                for parameter in group.get("parameters", [])
                            )
                        )
                        if v is not None
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

    def cleanup():
        for observer in observers:
            observer.stop()
        for observer in observers:
            observer.join()

    rospy.on_shutdown(cleanup)

    try:
        while len(active_observers := [observer for observer in observers if observer.is_alive()]):
            for observer in active_observers:
                observer.join(1)
    except KeyboardInterrupt as e:
        raise e
    finally:
        cleanup()
        


if __name__ == "__main__":
    rospy.init_node("task_generator_filewatcher")
    run()
