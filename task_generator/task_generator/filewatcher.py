#! /usr/bin/env python3

import functools
import json
import os
from typing import Any, Callable, List, Optional

import rospkg
import yaml

from rosros import rospify as rospy

import watchdog.observers
import watchdog.events

from task_generator.cfg import TaskGeneratorConfig

def observe(file: str, callback: watchdog.events.FileSystemEventHandler):
    observer = watchdog.observers.Observer()
    observer.schedule(callback, path=file, recursive=False)
    observer.start()
    return observer

def safe_callback(fn: Callable):
    def wrapper(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except KeyboardInterrupt as e:
            raise e
        except Exception as e:
            rospy.logwarn(f"Exception in callback: {e}")

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
        return ";".join(map(str, var))
    if isinstance(var, dict):
        return json.dumps(var)
    return var

def get_or_ignore(obj: dict, key: str) -> dict:
    return {key: obj.get(key)} if key in obj else {}

def set_ros_params(params: dict, prefix: str = ""):
    for key, value in params.items():
        if isinstance(value, dict):
            set_ros_params(value, f"{prefix}{key}/")
        else:
            rospy.set_param(f"{prefix}{key}", value)

def run(namespace: Optional[str] = None):

    if namespace is None:
        namespace = rospy.get_namespace()

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
                content = yaml.safe_load(f)

            rospy.logdebug("SENSING CHANGE OF TASK_MODE PARAMS")

            if 'ros__parameters' in content:
                set_ros_params(content['ros__parameters'])

        def on_modified(self, event):
            @safe_callback
            def callback():
                self.reconfigure()
            callback()

    observers = [
        observe(FILE_TASK_CONFIG, TaskConfigHandler())
    ]

    def cleanup():
        for observer in observers:
            observer.stop()
        for observer in observers:
            observer.join()

    rospy.on_shutdown(cleanup)

    try:
        while any(observer.is_alive() for observer in observers):
            for observer in observers:
                observer.join(1)
    except KeyboardInterrupt as e:
        raise e
    finally:
        cleanup()
        
if __name__ == "__main__":
    rospy.init_node("task_generator_filewatcher")
    run()
