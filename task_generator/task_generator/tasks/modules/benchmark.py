import datetime
import hashlib
import json
import pathlib
import yaml
import os
import time
import typing
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters, DescribeParameters
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Empty as EmptySrv
from task_generator.constants import Constants
from task_generator.constants.runtime import Configuration
from task_generator.tasks.modules import TM_Module
from task_generator.tasks.task_factory import TaskFactory
from arena_rclpy_mixins.ROSParamServer import ROSParamServer
import logging
from logging import FileHandler, StreamHandler, Formatter
from rclpy.validate_full_topic_name import validate_full_topic_name


class _Config(typing.NamedTuple):
    @classmethod
    def parse(cls, obj: typing.Dict):
        return cls(
            suite=cls.Suite(**obj["suite"]),
            contest=cls.Contest(**obj["contest"]),
            general=cls.General(**obj["general"]),
        )

    class Suite(typing.NamedTuple):
        config: str
        scale_episodes: float = 1

    class Contest(typing.NamedTuple):
        config: str

    class General(typing.NamedTuple):
        simulator: str

    suite: Suite
    contest: Contest
    general: General


class Suite(typing.NamedTuple):
    @classmethod
    def parse(cls, name: str, obj: typing.Dict, config_class=None):
        return cls(
            name=name,
            stages=[cls.Stage.parse(stage, config_class) for stage in obj["stages"]]
        )

    class Index(int):
        pass

    class Stage(typing.NamedTuple):
        name: str
        episodes: int
        robot: str
        map: str
        tm_robots: Constants.TaskMode.TM_Robots
        tm_obstacles: Constants.TaskMode.TM_Obstacles
        config: typing.Dict
        seed: int
        timeout: str

        @classmethod
        def _make_serializable(cls, item):
            if isinstance(item, dict):
                return {k: cls._make_serializable(v) for k, v in item.items()}
            elif isinstance(item, (list, tuple)):
                return [cls._make_serializable(i) for i in item]
            elif isinstance(item, (Constants.TaskMode.TM_Robots, Constants.TaskMode.TM_Obstacles)):
                return item.value
            elif hasattr(item, 'value'):
                return cls._make_serializable(item.value)
            elif hasattr(item, 'get_value'):
                return cls._make_serializable(item.get_value())
            elif hasattr(item, '__str__'):
                logging.getLogger("benchmark").debug(f"Converting unknown type {type(item)} to str: {str(item)}")
                return str(item)
            return item

        @classmethod
        def hash(cls, obj: typing.Dict) -> int:
            logger = logging.getLogger("benchmark")
            hashable_obj = {k: v for k, v in obj.items() if k != "config"}
            hashable_obj = cls._make_serializable(hashable_obj)
            try:
                return 0x7fffffff & int.from_bytes(
                    hashlib.sha1(json.dumps(hashable_obj).encode()).digest()[-4:], byteorder="big"
                )
            except Exception as e:
                logger.error(f"Hash failed: {e}, using fallback seed")
                return 0

        @classmethod
        def parse(cls, obj: typing.Dict, config_class=None) -> "Suite.Stage":
            if config_class is None:
                raise ValueError("Configuration class must be provided")
            logger = logging.getLogger("benchmark")
            if "tm_robots" in obj:
                if isinstance(obj["tm_robots"], str):
                    try:
                        obj["tm_robots"] = Constants.TaskMode.TM_Robots[obj["tm_robots"].upper()]
                    except KeyError:
                        logger.error(f"Invalid tm_robots value: {obj['tm_robots']}")
                        raise
                elif hasattr(obj["tm_robots"], 'value'):
                    obj["tm_robots"] = Constants.TaskMode.TM_Robots[obj["tm_robots"].value.upper()]
                elif hasattr(obj["tm_robots"], 'get_value'):
                    obj["tm_robots"] = Constants.TaskMode.TM_Obstacles[obj["tm_robots"].get_value().upper()]
                else:
                    logger.error(f"Invalid tm_robots type: {type(obj['tm_robots'])}")
                    raise ValueError(f"Invalid tm_robots type: {type(obj['tm_robots'])}")
            if "tm_obstacles" in obj:
                if isinstance(obj["tm_obstacles"], str):
                    try:
                        obj["tm_obstacles"] = Constants.TaskMode.TM_Obstacles[obj["tm_obstacles"].upper()]
                    except KeyError:
                        logger.error(f"Invalid tm_obstacles value: {obj['tm_obstacles']}")
                        raise
                elif hasattr(obj["tm_obstacles"], 'value'):
                    obj["tm_obstacles"] = Constants.TaskMode.TM_Obstacles[obj["tm_obstacles"].value.upper()]
                elif hasattr(obj["tm_obstacles"], 'get_value'):
                    obj["tm_obstacles"] = Constants.TaskMode.TM_Obstacles[obj["tm_obstacles"].get_value().upper()]
                else:
                    logger.error(f"Invalid tm_obstacles type: {type(obj['tm_obstacles'])}")
                    raise ValueError(f"Invalid tm_obstacles type: {type(obj['tm_obstacles'])}")
            obj.setdefault("timeout", str(config_class.Robot.TIMEOUT))
            obj.setdefault("seed", cls.hash(obj))
            return cls(**obj)

    name: str
    stages: typing.List[Stage]

    @property
    def min_index(self):
        return self.Index()

    @property
    def max_index(self) -> Index:
        return self.Index(len(self.stages) - 1)

    def config(self, index: Index) -> Stage:
        return self.stages[index]


class Contest(typing.NamedTuple):
    @classmethod
    def parse(cls, name: str, obj: dict):
        return cls(
            name=name,
            contestants=[cls.Contestant.parse(contestant) for contestant in obj["contestants"]]
        )

    class Index(int):
        pass

    class Contestant(typing.NamedTuple):
        name: str
        local_planner: str
        inter_planner: str
        agent_name: str = ""

        @classmethod
        def parse(cls, obj: typing.Dict) -> "Contestant":
            obj.setdefault("inter_planner", "navigate_w_replanning_time")
            return cls(**obj)

    name: str
    contestants: typing.List[Contestant]

    @property
    def min_index(self):
        return self.Index()

    @property
    def max_index(self) -> Index:
        return self.Index(len(self.contestants) - 1)

    def config(self, index: int) -> Contestant:
        return self.contestants[index]


class Mod_Benchmark(TM_Module):
    DIR = pathlib.Path(os.path.join(get_package_share_directory("arena_bringup"), "configs", "benchmark"))
    LOCK_FILE = "resume.lock"
    LOG_DIR = DIR / "logs"
    PARAM_SET_TIMEOUT = 5.0
    PARAM_SET_RETRIES = 3
    PARAM_SET_BACKOFF = 2.0
    RESET_RETRY_LIMIT = 5
    RESET_RETRY_DELAY = 10.0
    SERVICE_WAIT_TIMEOUT = 15.0

    _config: _Config
    _suite: Suite
    _contest: Contest
    _episode_index: int
    _runid: str
    _contest_index: Contest.Index
    _suite_index: Suite.Index
    _headless: int
    _config_class: typing.Any
    _primary_node: str
    _logger_object: logging.Logger = None

    @classmethod
    def _load_config(cls) -> _Config:
        with open(cls.DIR / "config.yaml") as f:
            return _Config.parse(yaml.safe_load(f))

    @classmethod
    def _load_contest(cls, contest: str) -> Contest:
        with open(cls.DIR / "contests" / contest) as f:
            return Contest.parse(pathlib.Path(contest).name.strip(".yaml"), yaml.safe_load(f))

    @classmethod
    def _load_suite(cls, suite: str, config_class):
        with open(cls.DIR / "suites" / suite) as f:
            return Suite.parse(pathlib.Path(suite).name.strip(".yaml"), yaml.safe_load(f), config_class)

    @classmethod
    def _resume(cls):
        with open(cls.DIR / cls.LOCK_FILE) as f:
            runid, contest, suite, headless = f.read().split(" ")
            return runid, Contest.Index(contest), Suite.Index(suite), int(headless)

    def _normalize_namespace(self, namespace: str) -> str:
        """Normalize namespace by removing extra slashes and ensuring proper format."""
        namespace = os.path.normpath(namespace)
        return os.path.join("/", namespace) if namespace else self.node.service_namespace()

    def _validate_parameters(self, node_name, param_names):
        logger = self._logger
        clean_node_name = self._normalize_namespace(node_name)
        service_name = os.path.join(clean_node_name, "describe_parameters")
        logger.debug(f"Creating client for service: {service_name}")
        describe_client = self.node.create_client(DescribeParameters, service_name)
        if not describe_client.wait_for_service(timeout_sec=self.SERVICE_WAIT_TIMEOUT):
            logger.warning(f"DescribeParameters service not available for {clean_node_name}")
            return []
        request = DescribeParameters.Request()
        request.names = param_names
        try:
            future = describe_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.PARAM_SET_TIMEOUT)
            if future.result():
                valid_params = [desc.name for desc in future.result().descriptors]
                logger.debug(f"Valid parameters for {clean_node_name}: {valid_params}")
                return valid_params
            else:
                logger.warning(f"Failed to describe parameters for {clean_node_name}")
                return []
        except Exception as e:
            logger.warning(f"Error validating parameters for {clean_node_name}: {e}")
            return []
        finally:
            self.node.destroy_client(describe_client)

    def _set_node_parameters(self, node_name, suite_config):
        logger = self._logger
        clean_node_name = self._normalize_namespace(node_name)
        logger.debug(f"Setting parameters for {clean_node_name}")

        params_to_set = [
            ('tm_robots', Parameter.Type.STRING, suite_config.tm_robots.value),
            ('tm_obstacles', Parameter.Type.STRING, suite_config.tm_obstacles.value)
        ]

        # Check if parameters are declared
        valid_params = self._validate_parameters(clean_node_name, [name for name, _, _ in params_to_set])
        if not all(name in valid_params for name, _, _ in params_to_set):
            logger.warning(f"Parameters {', '.join(name for name, _, _ in params_to_set if name not in valid_params)} not declared on {clean_node_name}. Please declare them in task_generator_node.py.")
            return False

        service_name = os.path.join(clean_node_name, "set_parameters")
        logger.debug(f"Creating client for service: {service_name}")
        set_client = self.node.create_client(SetParameters, service_name)
        if not set_client.wait_for_service(timeout_sec=self.SERVICE_WAIT_TIMEOUT):
            logger.warning(f"SetParameters service not available for {clean_node_name}")
            return False

        success = True
        for name, param_type, value in params_to_set:
            for attempt in range(self.PARAM_SET_RETRIES):
                try:
                    param = Parameter(name, param_type, value)
                    request = SetParameters.Request()
                    request.parameters = [param.to_parameter_msg()]
                    future = set_client.call_async(request)
                    rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.PARAM_SET_TIMEOUT)
                    if future.result() and all(r.successful for r in future.result().results):
                        logger.info(f"Set parameter {name}={value} on {clean_node_name}")
                        break
                    else:
                        logger.warning(f"Failed to set {name} on {clean_node_name}: {future.result().results[0].reason if future.result() else 'No result'}")
                        time.sleep(self.PARAM_SET_BACKOFF)
                except Exception as e:
                    logger.warning(f"Error setting {name} on {clean_node_name} (attempt {attempt+1}/{self.PARAM_SET_RETRIES}): {e}")
                    time.sleep(self.PARAM_SET_BACKOFF)
            else:
                logger.error(f"Failed to set {name} on {clean_node_name} after {self.PARAM_SET_RETRIES} attempts")
                success = False

        self.node.destroy_client(set_client)
        return success

    def _select_task_generator_node(self, node_list):
        logger = self._logger
        task_generator_nodes = []
        for item in node_list:
            if hasattr(item, 'name') and hasattr(item, 'namespace'):
                name, ns = item.name, item.namespace
            else:
                try:
                    name, ns = item
                except (TypeError, ValueError):
                    logger.debug(f"Invalid node list item: {item}")
                    continue
            if name.startswith("task_generator_node"):
                task_generator_nodes.append(self._normalize_namespace(os.path.join(ns, name)))

        if not task_generator_nodes:
            logger.error("No task_generator_node found. Check task_generator.launch.py and ensure the node is running.")
            return "/task_generator_node"
        if len(task_generator_nodes) > 1:
            logger.warning(f"Multiple task_generator_nodes detected: {task_generator_nodes}. Selecting first valid node. Ensure env_n=1 in start_arena.launch.py.")

        for node_name in task_generator_nodes:
            clean_node_name = self._normalize_namespace(node_name)
            service_name = os.path.join(clean_node_name, "reset_task")
            logger.debug(f"Checking service availability for {service_name}")
            client = self.node.create_client(EmptySrv, service_name)
            if not client.wait_for_service(timeout_sec=1.0):
                logger.debug(f"No reset_task service for {clean_node_name}")
                self.node.destroy_client(client)
                continue
            self.node.destroy_client(client)
            valid_params = self._validate_parameters(clean_node_name, ['tm_robots', 'tm_obstacles'])
            if 'tm_robots' in valid_params and 'tm_obstacles' in valid_params:
                logger.info(f"Selected task_generator_node: {clean_node_name} with valid parameters")
                return clean_node_name
            logger.debug(f"Node {clean_node_name} lacks required parameters: {valid_params}")

        logger.warning("No task_generator_node with required parameters and services found. Using first detected node.")
        return task_generator_nodes[0] if task_generator_nodes else self.node.service_namespace()

    def __init__(self, task, **kwargs):
        super().__init__(task, **kwargs)

        self._runid = f"t{int(time.time())}"
        # Log detected task_generator_nodes
        node_list = self.node.get_node_names_and_namespaces()
        task_generator_nodes = [self._normalize_namespace(os.path.join(ns, name)) for name, ns in node_list if name.startswith("task_generator_node")]
        logger = self._logger
        if len(task_generator_nodes) > 1:
            logger.warning(f"Multiple task_generator_nodes detected: {task_generator_nodes}. Selecting one with valid parameters and services. Ensure env_n=1 in start_arena.launch.py.")
            self._primary_node = self._select_task_generator_node(task_generator_nodes)
        elif not task_generator_nodes:
            logger.error("No task_generator_node found. Check task_generator.launch.py and ensure the node is running.")
            self._primary_node = self.node.service_namespace()
        else:
            self._primary_node = task_generator_nodes[0]
            logger.info(f"Single task_generator_node detected: {self._primary_node}")

        # self._config_class = Configuration(ROSParamServer(f'benchmark_param_server_{int(time.time())}'))
        self._config = self._load_config()
        self._suite = self._load_suite(suite=self._config.suite.config, config_class=self.node.conf)
        self._contest = self._load_contest(self._config.contest.config)
        self._episode_index = -1
        self._contest_index = self._contest.min_index
        self._suite_index = self._suite.min_index
        self._headless = 1

        os.makedirs(self.LOG_DIR, exist_ok=True)
        with open(self.LOG_DIR / f"{self._runid}.log", "w") as f:
            f.write(f"run {self._runid}\n")
            f.write(f"contest {self._contest.name}\n")
            f.write(f"suite {self._suite.name}\n")

        self._log_contest()
        self._log_suite()
        self._reincarnate()

    def before_reset(self):
        self._logger.debug("Before task reset")

    def after_reset(self):
        self._logger.debug(f"Episode: {self._episode_index + 1}")
        self._episode_index += 1
        episode_limit = int(self._suite.config(self._suite_index).episodes * self._config.suite.scale_episodes)
        if self._episode_index < episode_limit - 1:
            self._reset_task()
        else:
            self._episode_index = -1
            self.suite_index += 1

    def _reset_task(self, primary_node: str = None):
        logger = self._logger
        primary_node = self._normalize_namespace(primary_node or self._primary_node)
        logger.debug(f"Attempting task reset on {primary_node}")
        service_name = os.path.join(primary_node, "reset_task")

        try:
            validate_full_topic_name(service_name, is_service=True)
        except rclpy.exceptions.InvalidServiceNameException as e:
            logger.error(f"Invalid service name: {service_name}. Error: {str(e)}")
            return False

        reset_task_client = self.node.create_client(EmptySrv, service_name)
        if not reset_task_client.wait_for_service(timeout_sec=self.SERVICE_WAIT_TIMEOUT):
            logger.warning(f"Service {service_name} not available")
            return False
        request = EmptySrv.Request()
        try:
            future = reset_task_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.PARAM_SET_TIMEOUT)
            if future.result():
                logger.info(f"Task reset successfully on {service_name}")
                return True
            else:
                logger.warning(f"Task reset failed on {service_name}")
                return False
        except Exception as e:
            logger.warning(f"Reset task failed on {service_name}: {e}")
            return False
        finally:
            self.node.destroy_client(reset_task_client)

    @property
    def _logger(self) -> logging.Logger:
        if self._logger_object is None:
            handler = FileHandler(self.LOG_DIR / f"{self._runid}.log")
            handler.setFormatter(Formatter("%(asctime)s: %(levelname)s: %(message)s"))
            console_handler = StreamHandler()
            console_handler.setFormatter(Formatter("%(asctime)s: %(levelname)s: %(message)s"))
            logger = logging.getLogger("benchmark")
            logger.setLevel(logging.DEBUG)
            logger.addHandler(handler)
            logger.addHandler(console_handler)
            self._logger_object = logger
        return self._logger_object

    def _log_contest(self):
        self._logger.info(f"C [{1+self._contest_index}/{1+self._contest.max_index}] {self._contest.config(self._contest_index).name}")

    def _log_suite(self):
        self._logger.info(f"S [{1+self._suite_index}/{1+self._suite.max_index}] {self._suite.config(self._suite_index).name}")

    def _log_episode(self):
        if self._episode_index < 0:
            return
        episode_limit = int(self._suite.config(self._suite_index).episodes * self._config.suite.scale_episodes)
        self._logger.info(f"E [{1+self._episode_index}/{episode_limit}]")

    @property
    def contest_index(self) -> Contest.Index:
        return self._contest_index

    @contest_index.setter
    def contest_index(self, index: int):
        self._contest_index = Contest.Index(index)
        if self._contest_index > self._contest.max_index:
            self._logger.info("Benchmark completed")
        else:
            self._log_contest()
            self._reincarnate()

    @property
    def suite_index(self) -> Suite.Index:
        return self._suite_index

    @suite_index.setter
    def suite_index(self, index: int):
        self._suite_index = Suite.Index(index)
        if self._suite_index > self._suite.max_index:
            self._suite_index = self._suite.min_index
            self.contest_index += 1
        else:
            self._log_suite()
            self._reincarnate()

    @property
    def _episode(self) -> int:
        return self._episode_index

    @_episode.setter
    def _episode(self, episode: int):
        episode_limit = int(self._suite.config(self._suite_index).episodes * self._config.suite.scale_episodes)
        if episode >= episode_limit:
            self._episode_index = -1
            self.suite_index += 1
        else:
            self._episode_index = episode
            self._log_episode()

    def _reincarnate(self):
        logger = self._logger
        logger.debug("Starting reincarnation process")
        suite_config = self._suite.config(self._suite_index)
        logger.info(f"Transitioning to stage: {suite_config.name} (tm_robots={suite_config.tm_robots.value}, tm_obstacles={suite_config.tm_obstacles.value})")
        node_list = self.node.get_node_names_and_namespaces()
        task_generator_nodes = [self._normalize_namespace(os.path.join(ns, name)) for name, ns in node_list if name.startswith("task_generator_node")]
        logger.debug(f"Detected task_generator_nodes: {task_generator_nodes}")
        success = False
        selected_node = self._primary_node
        if len(task_generator_nodes) > 1:
            logger.warning(f"Multiple task_generator_nodes detected: {task_generator_nodes}. Selecting one with valid parameters and services.")
            selected_node = self._select_task_generator_node(task_generator_nodes)
        elif task_generator_nodes:
            selected_node = task_generator_nodes[0]

        selected_node = self.node.service_namespace()
        if self._set_node_parameters(selected_node, suite_config):
            logger.info(f"Stage setup complete for {suite_config.name} on {selected_node}")
            self._primary_node = selected_node
            success = True
        else:
            logger.error(f"Failed to set parameters for {suite_config.name} on {selected_node}")

        if not success:
            logger.error(f"Failed to set parameters for {suite_config.name} on any task_generator_node. Ensure tm_robots and tm_obstacles are declared in task_generator_node.py.")
        self._reset_task()
        self._episode = 0
