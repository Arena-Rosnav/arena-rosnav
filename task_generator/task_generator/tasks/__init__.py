
from .base_task import BaseTask # noqa
from .task_factory import TaskFactory # noqa

# add every new task mode so it gets imported
from .random import RandomTask  # noqa
from .scenario import ScenarioTask  # noqa
from .staged import StagedRandomTask  # noqa
from .parametrized import ParametrizedTask  # noqa
from .dynamic_map_random import DynamicMapRandomTask  # noqa
from .dynamic_map_staged import DynamicMapStagedRandomTask  # noqa