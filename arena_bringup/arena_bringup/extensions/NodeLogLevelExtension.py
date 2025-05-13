import logging
from typing import Literal

import launch
from launch_ros.actions import Node as NodeAction
from launch_ros.actions.node import NodeActionExtension
from launch_ros.utilities import plugin_support


class NodeLogLevelExtension(NodeActionExtension):
    NAME = "NodeLogLevelExtension"
    EXTENSION_POINT_VERSION = '0.1'

    def __init__(self):
        super(NodeActionExtension, self).__init__()
        plugin_support.satisfies_version(self.EXTENSION_POINT_VERSION, '^0.1')

    def prepare_for_execute(self, context: launch.LaunchContext, ros_specific_arguments: dict, node_action: NodeAction):
        global_log_level = context.launch_configurations.get('NodeLogLevelExtension_log_level', None)

        args = []

        if global_log_level is not None:
            args.append([launch.substitutions.TextSubstitution(text='--log-level')])
            args.append([launch.substitutions.TextSubstitution(text=global_log_level)])

        return args, ros_specific_arguments


class SetGlobalLogLevelAction(launch.Action):
    LOGLEVEL = Literal['debug', 'info', 'warn', 'error', 'fatal']
    _log_level: LOGLEVEL

    def __init__(self, log_level: LOGLEVEL, **kwargs) -> None:
        super().__init__(**kwargs)
        self._log_level = log_level

    @classmethod
    def str_to_level(cls, log_level: LOGLEVEL) -> int:
        if log_level == 'debug':
            return logging.DEBUG
        if log_level == 'info':
            return logging.INFO
        if log_level == 'warn':
            return logging.WARN
        if log_level == 'error':
            return logging.ERROR
        if log_level == 'fatal':
            return logging.FATAL
        return logging.NOTSET

    def execute(self, context: launch.LaunchContext):
        log_level_sub = launch.utilities.normalize_to_list_of_substitutions([self._log_level])
        log_level = launch.utilities.perform_substitutions(context, log_level_sub)
        context.launch_configurations['NodeLogLevelExtension_log_level'] = log_level
        # launch.logging.launch_config.level = self.str_to_level(self._log_level)
