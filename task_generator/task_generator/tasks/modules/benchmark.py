import datetime
import hashlib
import json
import pathlib
from task_generator.constants import Constants
from task_generator.constants.runtime import Configuration
from task_generator.tasks.modules import TM_Module
from task_generator.tasks.task_factory import TaskFactory
from arena_rclpy_mixins.ROSParamServer import ROSParamServer

import typing
import os
import yaml
import subprocess

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import arena_evaluation_msgs.srv as arena_evaluation_srvs

import logging

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

    class Index(int): ...

    class Stage(typing.NamedTuple):
        name: str
        episodes: int
        robot: str
        map: str
        tm_robots: Constants.TaskMode.TM_Robots
        tm_obstacles: Constants.TaskMode.TM_Obstacles
        config: typing.Dict

        seed: int
        timeout: float

        @classmethod
        def _make_serializable(cls, item):
            """Recursively convert non-serializable objects to JSON-serializable types."""
            if isinstance(item, dict):
                return {k: cls._make_serializable(v) for k, v in item.items()}
            elif isinstance(item, (list, tuple)):
                return [cls._make_serializable(i) for i in item]
            elif hasattr(item, 'value'):  # Handle ROSParam_impl
                return cls._make_serializable(item.value)
            elif isinstance(item, Constants.TaskMode.TM_Robots) or isinstance(item, Constants.TaskMode.TM_Obstacles):
                return item.value  # Convert enums to their string values
            return item

        @classmethod
        def hash(cls, obj: typing.Dict) -> int:
            """
            Hash json-serializable object to non-negative int32, excluding config field
            """
            # Debug: Log the types of obj fields
            print("Debug: obj fields and types:", {k: type(v).__name__ for k, v in obj.items()})
            hashable_obj = {k: v for k, v in obj.items() if k != "config"}
            # Convert non-serializable types
            hashable_obj = cls._make_serializable(hashable_obj)
            print("Debug: hashable_obj:", hashable_obj)
            return 0x7F_FF_FF_FF & int.from_bytes(
                hashlib.sha1(json.dumps(hashable_obj).encode()).digest()[-4:], byteorder="big"
            )

        @classmethod
        def parse(cls, obj: typing.Dict, config_class=None) -> "Suite.Stage":
            if config_class is None:
                raise ValueError("config_class must be provided to parse Suite.Stage")
            obj.setdefault("timeout", config_class.Robot.TIMEOUT)
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
    def parse(cls, name: str, obj: typing.Dict):
        return cls(
            name=name,
            contestants=[
                cls.Contestant.parse(contestant) for contestant in obj["contestants"]
            ],
        )

    class Index(int): ...

    class Contestant(typing.NamedTuple):
        name: str
        local_planner: str
        inter_planner: str
        agent_name: str = ""

        @classmethod
        def parse(cls, obj: typing.Dict) -> "Contest.Contestant":
            obj.setdefault("inter_planner", "bypass")
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

    DIR = pathlib.Path(
        os.path.join(get_package_share_directory("arena_bringup"), "configs", "benchmark")
    )
    LOCK_FILE = "resume.lock"
    LOG_DIR = DIR / "logs"
    TASK_GENERATOR_CONFIG = os.path.join(
        get_package_share_directory("arena_bringup"), "configs", "task_generator.yaml"
    )
    TASK_GENERATOR_CONFIG_BKUP = TASK_GENERATOR_CONFIG + ".bkup"

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
    _config_class: typing.Any  # Store the Config class from Configuration

    # CONFIGURATION

    @classmethod
    def _load_config(cls) -> _Config:
        with open(cls.DIR / "config.yaml") as f:
            return _Config.parse(yaml.safe_load(f))

    @classmethod
    def _load_contest(cls, contest: str) -> Contest:
        with open(cls.DIR / "contests" / contest) as f:
            return Contest.parse(
                pathlib.Path(contest).stem, yaml.safe_load(f)
            )

    @classmethod
    def _load_suite(cls, suite: str, config_class) -> Suite:
        with open(cls.DIR / "suites" / suite) as f:
            return Suite.parse(pathlib.Path(suite).stem, yaml.safe_load(f), config_class)

    @classmethod
    def _resume(cls) -> typing.Tuple[str, Contest.Index, Suite.Index, int]:
        with open(cls.DIR / cls.LOCK_FILE) as f:
            runid, contest, suite, headless = f.read().split(" ")
        return runid, Contest.Index(contest), Suite.Index(suite), int(headless)

    @classmethod
    def _taskgen_backup(cls):
        if os.path.exists(bkup_file := cls.TASK_GENERATOR_CONFIG + ".bkup"):
            return
        with open(cls.TASK_GENERATOR_CONFIG) as fr:
            with open(bkup_file, "w") as fw:
                fw.write(fr.read())

    @classmethod
    def _taskgen_write(cls, *configs: typing.Dict):

        def overwrite(source: typing.Dict, target: typing.Dict):
            for k, v in source.items():
                if isinstance(v, dict):
                    target.setdefault(k, dict())
                    overwrite(v, target[k])
                else:
                    target[k] = v
            return target

        with open(cls.TASK_GENERATOR_CONFIG, "r") as f:
            joint_config = yaml.safe_load(f)

        for config in configs:
            overwrite(config, joint_config)

        with open(cls.TASK_GENERATOR_CONFIG, "w") as f:
            yaml.dump(joint_config, f)

    @classmethod
    def _taskgen_restore(cls, cleanup: bool = False):
        with open(cls.TASK_GENERATOR_CONFIG_BKUP) as fr:
            with open(cls.TASK_GENERATOR_CONFIG, "w") as fw:
                fw.write(fr.read())
        if cleanup:
            os.remove(cls.TASK_GENERATOR_CONFIG_BKUP)

    # RUNTIME

    def __init__(self, node: Node = None, **kwargs):

        # Initialize or use provided node
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self._node = rclpy.create_node('benchmark_module')
        else:
            self._node = node

        # Initialize Configuration with ROSParamServer using a new node name
        self._config_class = Configuration(ROSParamServer('benchmark_param_server'))

        self._config = self._load_config()
        self._suite = self._load_suite(self._config.suite.config, self._config_class)
        self._contest = self._load_contest(self._config.contest.config)

        self._requires_restart = False

        # Check if resuming from previous run
        benchmark_resume = self._node.get_parameter_or('benchmark_resume', rclpy.Parameter('benchmark_resume', rclpy.Parameter.Type.BOOL, False)).get_parameter_value().bool_value

        # first run
        if not benchmark_resume:
            self._taskgen_backup()
            self._runid = f"{self._contest.name}_{datetime.datetime.now().strftime('%y-%m-%d_%H-%M-%S')}"
            self._contest_index = self._contest.min_index
            self._suite_index = self._suite.min_index
            
            headless_param = self._node.get_parameter_or('headless', rclpy.Parameter('headless', rclpy.Parameter.Type.INTEGER, 1))
            self._headless = headless_param.get_parameter_value().integer_value

            self._taskgen_backup()
            with open(self.TASK_GENERATOR_CONFIG_BKUP) as f:
                base_config = f.read()

            os.makedirs(self.LOG_DIR, exist_ok=True)
            with open(self.LOG_DIR / f"{self._runid}.log", "w") as f:
                f.write(f"run {self._runid}\n")
                f.write(
                    f"of contest {self._contest.name} with {len(self._contest.contestants)} contestants\n"
                )
                f.write(
                    f"on suite {self._suite.name} with {len(self._suite.stages)} stages\n"
                )
                f.write(
                    f"total of {len(self._contest.contestants) * sum([int(self._config.suite.scale_episodes * self._suite.config(Suite.Index(index)).episodes) for index in range(self._suite.min_index, self._suite.max_index+1)])} episodes\n"
                )
                f.write("\n")
                f.write(f"Simulator: {self._config.general.simulator}\n")
                f.write(f"Base Config: {json.dumps(base_config)}\n")
                f.write(80 * "=" + "\n")
                f.write("\n")

            self._log_contest()
            self._log_suite()

            self._requires_restart = True
            return self._reincarnate()

        self._runid, contest_index, suite_index, headless = self._resume()

        self._contest_index, self._suite_index, self._headless = (
            contest_index,
            suite_index,
            headless,
        )
        self._episode_index = -1

    def before_reset(self):
        pass

    def after_reset(self):
        self._episode_index += 1

    _logger_object: logging.Logger

    @property
    def _logger(self) -> logging.Logger:
        if not hasattr(self, "_logger_object"):
            handler = logging.FileHandler(self.LOG_DIR / f"{self._runid}.log")
            handler.setFormatter(logging.Formatter("%(created)f: %(message)s"))

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
            return  # pre-init
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

            self._logger.debug(f"contestant change requires restart")
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

            if new_config.map != old_config.map:
                self._logger.debug(f"map change - will use service calls")
                # No restart needed, just mark for service calls

            if new_config.robot != old_config.robot:
                self._logger.debug(f"robot change - will use service calls")
                # No restart needed, just mark for service calls

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

    def _reincarnate(self):
        with open(self.DIR / self.LOCK_FILE, "w") as f:
            f.write(
                f"{self._runid} {self._contest_index} {self._suite_index} {self._headless}"
            )

        config = self._config
        contest_config = self._contest.config(self._contest_index)
        suite_config = self._suite.config(self._suite_index)

        self._taskgen_restore()
        self._taskgen_write(
            {
                "episodes": -1,
                "RANDOM": {
                    "seed": suite_config.seed ^ Suite.Stage.hash({"": self._runid})
                },
            },
            suite_config.config,
        )

        record_data_dir = f"{self._runid}/{contest_config.name}/{suite_config.name}"

        if self._requires_restart:
            # Initial bootstrap - launch the simulator with initial config
            self._logger.info("Bootstrapping ROS2 system with initial configuration")
            
            launch_args = [
                "ros2", "launch", "arena_bringup", "start_arena.launch.py",
                f"tm_modules:=benchmark",
                f"benchmark_resume:=true",
                f"record_data:=true",
                f"record_data_dir:={record_data_dir}",
                f"simulator:={config.general.simulator}",
                f"timeout:={suite_config.timeout}",
                f"headless:={self._headless}",
                # contest
                f"inter_planner:={contest_config.inter_planner}",
                f"local_planner:={contest_config.local_planner}",
                # suite
                f"model:={suite_config.robot}",
                f"map_file:={suite_config.map}",
                f"tm_robots:={suite_config.tm_robots}",
                f"tm_obstacles:={suite_config.tm_obstacles}",
                f"agent_name:={contest_config.agent_name}",
            ]
            
            subprocess.Popen(launch_args, start_new_session=True)
            
            import sys
            sys.exit(0)

        else:
            self._change_world_via_services(record_data_dir, suite_config)
            self._episode = 0

    def _change_world_via_services(self, record_data_dir: str, suite_config):
        """Change world configuration via service calls instead of restarting"""
        

        change_dir_client = self._node.create_client(
            arena_evaluation_srvs.ChangeDirectory,
            f"/{suite_config.robot}/change_directory"
        )
        
        if change_dir_client.wait_for_service(timeout_sec=5.0):
            request = arena_evaluation_srvs.ChangeDirectory.Request()
            request.directory = record_data_dir
            
            future = change_dir_client.call_async(request)
            rclpy.spin_until_future_complete(self._node, future)
            
            if future.result() is not None:
                self._logger.info(f"Successfully changed recording directory to: {record_data_dir}")
            else:
                self._logger.error("Failed to change recording directory")
        else:
            self._logger.error("change_directory service not available")
        
        # Add other service calls here for changing map, robot config, etc.

    def _suicide(self):
        """Clean shutdown of the benchmark module"""
        try:
            self._logger.info("Shutting down benchmark module")
            
            if hasattr(self, '_node') and self._node is not None:
                try:
                    self._node.destroy_node()
                    self._node = None
                except Exception as e:
                    self._logger.warning(f"Error destroying node: {e}")
            
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception as e:
                    self._logger.warning(f"Error shutting down rclpy: {e}")
                    
        except Exception as e:
            self._logger.error(f"Error during shutdown: {e}")

    def __del__(self):
        """Cleanup when object is destroyed"""
        if hasattr(self, '_node') and self._node is not None:
            try:
                self._node.destroy_node()
            except:
                pass