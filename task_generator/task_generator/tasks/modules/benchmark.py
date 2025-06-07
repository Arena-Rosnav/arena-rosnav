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
from ament_index_python.packages import get_package_share_directory
import arena_evaluation_msgs.srv as arena_evaluation_srvs
from std_srvs.srv import Empty as EmptySrv
from task_generator.constants import Constants
from task_generator.constants.runtime import Configuration
from task_generator.tasks.modules import TM_Module
from task_generator.tasks.task_factory import TaskFactory
from arena_rclpy_mixins.ROSParamServer import ROSParamServer
import logging
from logging import FileHandler, Formatter
import geometry_msgs.msg

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

    class Index(int): pass

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
            elif isinstance(item, Constants.TaskMode.TM_Robots) or isinstance(item, Constants.TaskMode.TM_Obstacles):
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
            logger.debug(f"obj fields and types: {{k: type(v).__name__ for k, v in obj.items()}}")
            hashable_obj = {k: v for k, v in obj.items() if k != "config"}
            hashable_obj = cls._make_serializable(hashable_obj)
            logger.debug(f"hashable_obj: {hashable_obj}")
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
                    logger.debug(f"Converted ROSParam_impl tm_robots to {obj['tm_robots']}")
                elif hasattr(obj["tm_robots"], 'get_value'):
                    obj["tm_robots"] = Constants.TaskMode.TM_Robots[obj["tm_robots"].get_value().upper()]
                    logger.debug(f"Converted ROSParam_impl tm_robots to {obj['tm_robots']}")
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
                    logger.debug(f"Converted ROSParam_impl tm_obstacles to {obj['tm_obstacles']}")
                elif hasattr(obj["tm_obstacles"], 'get_value'):
                    obj["tm_obstacles"] = Constants.TaskMode.TM_Obstacles[obj["tm_obstacles"].get_value().upper()]
                    logger.debug(f"Converted ROSParam_impl tm_obstacles to {obj['tm_obstacles']}")
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

    class Index(int): pass

    class Contestant(typing.NamedTuple):
        name: str
        local_planner: str
        inter_planner: str
        agent_name: str = ""

        @classmethod
        def parse(cls, obj: typing.Dict) -> "Contest.Contestant":
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

    def config(self, index: Index) -> Contestant:
        return self.contestants[index]

@TaskFactory.register_module(Constants.TaskMode.TM_Module.BENCHMARK)
class Mod_Benchmark(TM_Module):
    DIR = pathlib.Path(os.path.join(get_package_share_directory("arena_bringup"), "configs", "benchmark"))
    LOCK_FILE = "resume.lock"
    LOG_DIR = DIR / "logs"
    TASK_GENERATOR_CONFIG = os.path.join(get_package_share_directory("arena_bringup"), "configs/task_generator.yaml")
    TASK_GENERATOR_CONFIG_BKUP = TASK_GENERATOR_CONFIG + ".bkup"
    TEMP_ROBOT_SETUP_DIR = DIR / "temp"
    MAX_RESTARTS = 5

    _config: _Config
    _suite: Suite
    _contest: Contest
    _episode_index: int
    _runid: str
    _contest_index: Contest.Index
    _suite_index: Suite.Index
    _headless: int
    _requires_restart: bool
    _node: Node
    _config_class: typing.Any
    _restart_count: int

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

    @classmethod
    def _taskgen_backup(cls):
        if os.path.exists(cls.TASK_GENERATOR_CONFIG_BKUP):
            return
        try:
            with open(cls.TASK_GENERATOR_CONFIG) as fr:
                with open(cls.TASK_GENERATOR_CONFIG_BKUP, "w") as fw:
                    fw.write(fr.read())
            logging.getLogger("benchmark").debug("Backed up task_generator.yaml")
        except Exception as e:
            logging.getLogger("benchmark").error(f"Failed to backup task_generator.yaml: {e}")

    @classmethod
    def _taskgen_write(cls, *configs: typing.Dict):
        def overwrite(source: typing.Dict, target: typing.Dict):
            for k, v in source.items():
                if isinstance(v, dict):
                    target.setdefault(k, {})
                    overwrite(v, target[k])
                else:
                    target[k] = v

        try:
            with open(cls.TASK_GENERATOR_CONFIG, "r") as f:
                joint_config = yaml.safe_load(f) or {"task_generator_node": {"ros__parameters": {"task": {}, "hunav": {}}}}
        except Exception as e:
            logging.getLogger("benchmark").error(f"Failed to read task_generator.yaml: {e}")
            joint_config = {"task_generator_node": {"ros__parameters": {"task": {}, "hunav": {}}}}

        task_params = joint_config.setdefault("task_generator_node", {}).setdefault("ros__parameters", {}).setdefault("task", {})
        for config in configs:
            overwrite(config, task_params)

        try:
            with open(cls.TASK_GENERATOR_CONFIG, "w") as f:
                yaml.safe_dump(joint_config, f)
            logging.getLogger("benchmark").debug(f"Updated task_generator.yaml: {joint_config}")
        except IOError as e:
            logging.getLogger("benchmark").error(f"Failed to write task_generator.yaml: {e}")

    @classmethod
    def _taskgen_restore(cls, cleanup: bool = False):
        try:
            if os.path.exists(cls.TASK_GENERATOR_CONFIG_BKUP):
                with open(cls.TASK_GENERATOR_CONFIG_BKUP) as fr:
                    with open(cls.TASK_GENERATOR_CONFIG, "w") as fw:
                        fw.write(fr.read())
                if cleanup:
                    os.remove(cls.TASK_GENERATOR_CONFIG_BKUP)
            logging.getLogger("benchmark").debug("Restored task_generator.yaml")
        except IOError as e:
            logging.getLogger("benchmark").error(f"Failed to restore task_generator.yaml: {e}")

    def __init__(self, node: Node = None, **kwargs):
        if node is None:
            if not rclpy.ok():
                try:
                    rclpy.init()
                except Exception as e:
                    logging.getLogger("benchmark").error(f"Failed to initialize rclpy: {e}")
                    raise
            node_name = f'benchmark_module_{int(time.time())}'
            self._node = Node(node_name)
        else:
            self._node = node

        self._config_class = Configuration(ROSParamServer(f'benchmark_param_server_{int(time.time())}'))
        self._config = self._load_config()
        self._suite = self._load_suite(suite=self._config.suite.config, config_class=self._config_class)
        self._contest = self._load_contest(self._config.contest.config)
        self._requires_restart = False
        self._restart_count = 0

        benchmark_resume = self._node.get_parameter_or(
            'benchmark_resume',
            Parameter('benchmark_resume', Parameter.Type.BOOL, False)
        ).get_parameter_value().bool_value

        if not benchmark_resume:
            self._taskgen_backup()
            self._runid = f"{self._contest.name}_{datetime.datetime.now().strftime('%y-%m-%d_%H-%M-%S')}"
            self._contest_index = self._contest.min_index
            self._suite_index = self._suite.min_index
            
            headless_param = self._node.get_parameter_or(
                'headless',
                Parameter('headless', Parameter.Type.INTEGER, 1)
            )
            self._headless = headless_param.get_parameter_value().integer_value

            os.makedirs(self.LOG_DIR, exist_ok=True)
            with open(self.LOG_DIR / f"{self._runid}.log", "w") as f:
                f.write(f"run {self._runid}\n")
                f.write(f"of contest {self._contest.name} with {len(self._contest.contestants)} contestants\n")
                f.write(f"on suite {self._suite.name} with {len(self._suite.stages)} stages\n")
                f.write(f"total of {len(self._contest.contestants) * sum([int(self._config.suite.scale_episodes * self._suite.config(Suite.Index(index)).episodes) for index in range(self._suite.min_index, self._suite.max_index+1)])} episodes\n")
                f.write("\n")
                f.write(f"Simulator: {self._config.general.simulator}\n")
                f.write(80 * "=" + "\n")
                f.write("\n")

            self._log_contest()
            self._log_suite()
            self._requires_restart = True
            return self._reincarnate()

        try:
            self._runid, contest_index, suite_index, headless = self._resume()
            self._contest_index, self._suite_index, self._headless = (
                contest_index,
                suite_index,
                headless,
            )
            self._episode_index = -1
        except FileNotFoundError:
            self._logger.error("Resume lock file not found, starting new benchmark")
            self.__init__(node=self._node, **kwargs)

    def before_reset(self):
        self._logger.debug("Before task reset")

    def after_reset(self):
        self._episode_index += 1
        self._logger.info(f"After task reset, episode: {self._episode_index}")

    _logger_object: logging.Logger

    @property
    def _logger(self) -> logging.Logger:
        if not hasattr(self, "_logger_object"):
            handler = FileHandler(self.LOG_DIR / f"{self._runid}.log")
            handler.setFormatter(Formatter("%(asctime)s: %(levelname)s: %(message)s"))
            logger = logging.getLogger("benchmark")
            logger.setLevel(logging.DEBUG)
            logger.addHandler(handler)
            self._logger_object = logger
        return self._logger_object

    def _log_contest(self):
        self._logger.info(
            f"\tC [{1+self._contest_index:0>{len(str(1+self._contest.max_index))}}/{1+self._contest.max_index}] {self._contest.config(self._contest_index).name}"
        )

    def _log_suite(self):
        self._logger.info(
            f"\t\tS [{1+self._suite_index:0>{len(str(1+self._suite.max_index))}}/{1+self._suite.max_index}] {self._suite.config(self._suite_index).name}"
        )

    def _log_episode(self):
        if self._episode_index < 0:
            return
        episode_limit = int(
            self._suite.config(self._suite_index).episodes
            * self._config.suite.scale_episodes
        )
        self._logger.info(
            f"\t\t\tE [{1+self._episode_index:0>{len(str(episode_limit))}}/{episode_limit}]"
        )

    @property
    def contest_index(self) -> Contest.Index:
        return self._contest_index

    @contest_index.setter
    def contest_index(self, index: int):
        self._contest_index = Contest.Index(index)
        if self._contest_index > self._contest.max_index:
            self._logger.info("BENCHMARK COMPLETED SUCCESSFULLY")
            os.remove(self.DIR / self.LOCK_FILE)
            self._taskgen_restore(cleanup=True)
            self._suicide()
        else:
            self._log_contest()
            self._requires_restart = True
            self._reincarnate()

    @property
    def suite_index(self) -> Suite.Index:
        return self._suite_index

    @suite_index.setter
    def suite_index(self, index: int):
        old_config = self._suite.config(self._suite_index)
        self._suite_index = Suite.Index(index)
        if self._suite_index > self._suite.max_index:
            self._suite_index = self._suite.min_index
            self.contest_index += 1
        else:
            self._log_suite()
            new_config = self._suite.config(self._suite_index)
            if new_config.map != old_config.map or new_config.robot != old_config.robot:
                self._requires_restart = True
            self._reincarnate()

    @property
    def _episode(self) -> int:
        return self._episode_index

    @_episode.setter
    def _episode(self, episode: int):
        if episode >= int(
            self._suite.config(self._suite_index).episodes
            * self._config.suite.scale_episodes
        ):
            self._episode_index = 0
            self.suite_index += 1
        else:
            self._episode_index = episode
            self._log_episode()

    def _create_temp_robot_setup_file(self, record_data_dir: str, robot: str) -> str:
        os.makedirs(self.TEMP_ROBOT_SETUP_DIR, exist_ok=True)
        temp_file = self.TEMP_ROBOT_SETUP_DIR / f"robot_setup_{int(time.time())}.yaml"
        config = {
            "robot": robot,
            "record_data_dir": record_data_dir
        }
        with open(temp_file, "w") as f:
            yaml.dump(config, f)
        self._logger.debug(f"Created temp robot setup: {temp_file}")
        return str(temp_file)

    def _check_duplicate_nodes(self, robot: str) -> bool:
        """Check for duplicate navigation nodes to prevent re-launching."""
        self._logger.debug("Checking for duplicate navigation nodes")
        try:
            node_list = self._node.get_node_names_and_namespaces()
            lifecycle_manager = f"/task_generator_node/{robot}/lifecycle_manager_navigation"
            count = sum(1 for name, ns in node_list if f"{ns}/{name}" == lifecycle_manager)
            if count > 1:
                self._logger.warning(f"Found {count} instances of {lifecycle_manager}")
                return True
            self._logger.debug("No duplicate navigation nodes found")
            return False
        except Exception as e:
            self._logger.error(f"Failed to check nodes: {e}")
            return False

    def _reincarnate(self):
        if self._restart_count >= self.MAX_RESTARTS:
            self._logger.error(f"Reached maximum restarts ({self.MAX_RESTARTS}), aborting")
            self._suicide()
            raise RuntimeError("Max restarts reached")

        self._restart_count += 1
        self._logger.info(f"Stage transition attempt {self._restart_count}/{self.MAX_RESTARTS}")

        with open(self.DIR / self.LOCK_FILE, "w") as f:
            f.write(f"{self._runid} {self._contest_index} {self._suite_index} {self._headless}")

        config = self._config
        contest_config = self._contest.config(self._contest_index)
        suite_config = self._suite.config(self._suite_index)

        self._logger.debug(f"Stage config: {suite_config._asdict()}")

        # Update task_generator.yaml
        self._taskgen_restore()
        self._taskgen_write(
            {
                "episodes": -1,
                "RANDOM": {
                    "seed": int(suite_config.seed) ^ Suite.Stage.hash({"": self._runid})
                },
                "tm_robots": suite_config.tm_robots.value,
                "tm_obstacles": suite_config.tm_obstacles.value,
                "model": suite_config.robot,
                "map_file": suite_config.map,
                "SCENARIO": suite_config.config.get("SCENARIO", {})
            },
            suite_config.config,
        )

        record_data_dir = f"{self._runid}/{contest_config.name}/{suite_config.name}"
        self._logger.debug(f"Record data dir: {record_data_dir}")

        if self._requires_restart and not self._check_duplicate_nodes(suite_config.robot):
            self._logger.info("Configuring stage via parameters and services")
            self._configure_stage(record_data_dir, suite_config)
            self._requires_restart = False
        else:
            self._logger.debug("Skipping stage configuration due to no restart needed or duplicates detected")
            self._episode = 0

    def _configure_stage(self, record_data_dir: str, suite_config):
        temp_setup_file = self._create_temp_robot_setup_file(record_data_dir, suite_config.robot)
        self._logger.debug(f"Created temporary robot setup file: {temp_setup_file}")

        # Set parameters
        try:
            self._node.set_parameters([
                Parameter('model', Parameter.Type.STRING, suite_config.robot),
                Parameter('map_file', Parameter.Type.STRING, suite_config.map),
                Parameter('tm_robots', Parameter.Type.STRING, suite_config.tm_robots.value),
                Parameter('tm_obstacles', Parameter.Type.STRING, suite_config.tm_obstacles.value)
            ])
            self._logger.info(f"Set parameters: model={suite_config.robot}, map_file={suite_config.map}, tm_robots={suite_config.tm_robots.value}, tm_obstacles={suite_config.tm_obstacles.value}")
        except Exception as e:
            self._logger.error(f"Failed to set parameters: {e}")

        # Change recording directory
        change_dir_client = self._node.create_client(
            arena_evaluation_srvs.ChangeDirectory,
            f"/task_generator_node/{suite_config.robot}/change_directory"
        )
        if change_dir_client.wait_for_service(timeout_sec=10.0):
            request = arena_evaluation_srvs.ChangeDirectory.Request()
            request.directory = record_data_dir
            try:
                future = change_dir_client.call_async(request)
                rclpy.spin_until_future_complete(self._node, future)
                if future.result():
                    self._logger.info(f"Changed recording directory to: {record_data_dir}")
                else:
                    self._logger.error("Failed to change recording directory")
            except Exception as e:
                self._logger.error(f"Change directory service failed: {e}")
        else:
            self._logger.error(f"Service /task_generator_node/{suite_config.robot}/change_directory not available")

        # Check nodes before reset
        self._logger.debug("Checking nodes before reset_task")
        try:
            node_list = self._node.get_node_names_and_namespaces()
            for name, ns in node_list:
                self._logger.debug(f"Node: {name} in namespace {ns}")
        except Exception as e:
            self._logger.error(f"Failed to list nodes: {e}")

        # Reset task
        reset_task_client = self._node.create_client(
            EmptySrv,
            "/task_generator_node/reset_task"
        )
        if reset_task_client.wait_for_service(timeout_sec=10.0):
            request = EmptySrv.Request()
            try:
                future = reset_task_client.call_async(request)
                rclpy.spin_until_future_complete(self._node, future)
                if future.result():
                    self._logger.info("Task reset successfully")
                else:
                    self._logger.error("Failed to reset task")
            except Exception as e:
                self._logger.error(f"Reset task service failed: {e}")
        else:
            self._logger.error("Service /task_generator_node/reset_task not available")

        # Check nodes after reset
        self._logger.debug("Checking nodes after reset_task")
        try:
            node_list = self._node.get_node_names_and_namespaces()
            lifecycle_manager = f"/task_generator_node/{suite_config.robot}/lifecycle_manager_navigation"
            count = sum(1 for name, ns in node_list if f"{ns}/{name}" == lifecycle_manager)
            if count > 1:
                self._logger.warning(f"Duplicate nodes detected: {count} instances of {lifecycle_manager}")
            for name, ns in node_list:
                self._logger.debug(f"Node: {name} in namespace {ns}")
        except Exception as e:
            self._logger.error(f"Failed to list nodes: {e}")

        # Verify goal publication
        self._logger.debug("Checking goal pose publication")
        goal_publisher = self._node.create_publisher(
            geometry_msgs.msg.PoseStamped,
            f"/task_generator_node/{suite_config.robot}/goal_pose",
            10
        )
        if goal_publisher.get_subscription_count() > 0:
            self._logger.debug("Goal pose topic has subscribers")
        else:
            self._logger.warning("No subscribers to goal pose topic")

    def _suicide(self):
        try:
            self._logger.info("Shutting down benchmark module")
            if self._node:
                self._node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            self._logger.error(f"Shutdown error: {e}")

    def __del__(self):
        if hasattr(self, '_node') and self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass