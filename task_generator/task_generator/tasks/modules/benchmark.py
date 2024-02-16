import datetime
import hashlib
import json
import pathlib
from task_generator.constants import Config, Constants
from task_generator.shared import Namespace, rosparam_get
from task_generator.tasks.modules import TM_Module
from task_generator.tasks.task_factory import TaskFactory

import typing
import os
import yaml
import subprocess

import rospkg
import rospy
import arena_evaluation_msgs.srv as arena_evaluation_srvs

import logging

def _get_rosmaster_pid() -> int:
    try:
        return int(subprocess.check_output(["ps", "-C", "rosmaster", "-o", "pid", "h"]).decode())
    except Exception as e:
        raise RuntimeError("could not determine rosmaster pid") from e

class _Config(typing.NamedTuple):

    @classmethod
    def parse(cls, obj: typing.Dict):
        return cls(
            suite = cls.Suite(**obj["suite"]),
            contest = cls.Contest(**obj["contest"]),
            general = cls.General(**obj["general"])
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
    def parse(cls, name: str, obj: typing.Dict):
        return cls(
            name = name,
            stages = [
                cls.Stage.parse(stage)
                for stage
                in obj["stages"]
            ]
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
        def hash(cls, obj: typing.Dict) -> int:
            """
            hash json-serializable object to non-negative int32
            """
            return 0x7f_ff_ff_ff & int.from_bytes(
                hashlib.sha1(json.dumps(obj).encode()).digest()[-4:],
                byteorder="big"
            )

        @classmethod
        def parse(cls, obj: typing.Dict) -> "Suite.Stage":
            obj.setdefault("timeout", Config.Robot.TIMEOUT)
            obj.setdefault("seed", cls.hash(obj))
            return cls(**obj)
        

    name: str
    stages: typing.List[Stage]

    @property
    def min_index(self):
        return self.Index()

    @property
    def max_index(self) -> Index:
        return self.Index(len(self.stages)-1)
    
    def config(self, index: Index) -> Stage:
        return self.stages[index]

class Contest(typing.NamedTuple):

    @classmethod
    def parse(cls, name: str, obj: typing.Dict):
        return cls(
            name = name,
            contestants = [
                cls.Contestant.parse(contestant)
                for contestant
                in obj["contestants"]
            ]
        )

    class Index(int): ...

    class Contestant(typing.NamedTuple):
        name: str
        local_planner: str
        inter_planner: str

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
        return self.Index(len(self.contestants)-1)
    
    def config(self, index: Index) -> Contestant:
        return self.contestants[index]

@TaskFactory.register_module(Constants.TaskMode.TM_Module.BENCHMARK)
class Mod_Benchmark(TM_Module):

    DIR: Namespace = Namespace(
        os.path.join(
                rospkg.RosPack().get_path("arena_bringup"),
                "configs",
                "benchmark"
            )
        )
    LOCK_FILE = "resume.lock"
    LOG_DIR = DIR("logs")
    TASK_GENERATOR_CONFIG = os.path.join(
        rospkg.RosPack().get_path("arena_bringup"),
        "configs",
        "task_generator.yaml"
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

    # CONFIGURATION

    @classmethod
    def _load_config(cls) -> _Config:
        with open(cls.DIR("config.yaml")) as f:
            return _Config.parse(yaml.load(f, yaml.FullLoader))

    @classmethod
    def _load_contest(cls, contest: str) -> Contest:
        with open(cls.DIR("contests", contest)) as f:
            return Contest.parse(pathlib.Path(contest).stem, yaml.load(f, yaml.FullLoader))
        
    @classmethod
    def _load_suite(cls, suite: str) -> Suite:
        with open(cls.DIR("suites", suite)) as f:
            return Suite.parse(pathlib.Path(suite).stem, yaml.load(f, yaml.FullLoader))
    
    @classmethod
    def _resume(cls) -> typing.Tuple[str, Contest.Index, Suite.Index, int]:
        with open(cls.DIR(cls.LOCK_FILE)) as f:
            runid, contest, suite, headless = f.read().split(" ")
        return runid, Contest.Index(contest), Suite.Index(suite), int(headless)


    @classmethod
    def _taskgen_backup(cls):
        if os.path.exists(bkup_file := cls.TASK_GENERATOR_CONFIG + ".bkup"): return
        with open(cls.TASK_GENERATOR_CONFIG) as fr:
            with open(bkup_file, "w") as fw:
                fw.write(fr.read())

    @classmethod
    def _taskgen_write(cls, *configs: typing.Dict):

        def overwrite(source: typing.Dict, target: typing.Dict):
            for k,v in source.items():
                if isinstance(v, dict):
                    target.setdefault(k, dict())
                    overwrite(v, target[k])
                else:
                    target[k] = v
            return target

        with open(cls.TASK_GENERATOR_CONFIG, "r") as f:
            joint_config = yaml.load(f, yaml.FullLoader)

        for config in configs:
            overwrite(config, joint_config)        
        
        # yaml has problems with r+
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

    def __init__(self, **kwargs):
        
        self._config = self._load_config()
        self._suite = self._load_suite(self._config.suite.config)
        self._contest = self._load_contest(self._config.contest.config)

        self._requires_restart = False

        # first run
        if not rosparam_get(bool, "benchmark_resume", False):
            self._taskgen_backup()
            self._runid = f"{self._contest.name}_{datetime.datetime.now().strftime('%y-%m-%d_%H-%M-%S')}"
            self._contest_index = self._contest.min_index
            self._suite_index = self._suite.min_index
            self._headless = rosparam_get(int, "headless", 1)

            self._taskgen_backup()
            with open(self.TASK_GENERATOR_CONFIG_BKUP) as f:
                base_config = f.read() 

            os.makedirs(self.LOG_DIR, exist_ok=True)
            with open(self.LOG_DIR(f"{self._runid}.log"), "w") as f:
                f.write(f"run {self._runid}\n")
                f.write(f"of contest {self._contest.name} with {len(self._contest.contestants)} contestants\n")
                f.write(f"on suite {self._suite.name} with {len(self._suite.stages)} stages\n")
                f.write(f"total of {len(self._contest.contestants) * sum([int(self._config.suite.scale_episodes * self._suite.config(Suite.Index(index)).episodes) for index in range(self._suite.min_index, self._suite.max_index+1)])} episodes\n")
                f.write("\n")
                f.write(f"Simulator: {self._config.general.simulator}\n")
                f.write(f"Base Config: {json.dumps(base_config)}\n")
                f.write(80*"=" + "\n")
                f.write("\n")

            self._log_contest()
            self._log_suite()

            self._requires_restart = True
            return self._reincarnate()

        self._runid, contest_index, suite_index, headless = self._resume()

        self._contest_index, self._suite_index, self._headless = contest_index, suite_index, headless
        self._episode = -1

    def before_reset(self):
        pass
    
    def after_reset(self):
        self._episode += 1


    _logger_object: logging.Logger

    @property
    def _logger(self) -> logging.Logger:
        if not hasattr(self, "_logger_object"):
            handler = logging.FileHandler(self.LOG_DIR(f"{self._runid}.log"))
            handler.setFormatter(logging.Formatter('%(created)f: %(message)s'))
            
            logger = logging.getLogger("benchmark")
            logger.setLevel(logging.DEBUG)
            logger.addHandler(handler)

            self._logger_object = logger
        
        return self._logger_object
    
    def _log_contest(self):
        self._logger.info(f"\tC [{1+self.contest_index:0>{len(str(1+self._contest.max_index))}}/{1+self._contest.max_index}] {self._contest.config(self._contest_index).name}")

    def _log_suite(self):
        self._logger.info(f"\t\tS [{1+self.suite_index:0>{len(str(1+self._suite.max_index))}}/{1+self._suite.max_index}] {self._suite.config(self._suite_index).name}")

    def _log_episode(self):
        if self._episode < 0: return #pre-init
        episode_limit = int(self._suite.config(self._suite_index).episodes * self._config.suite.scale_episodes)
        self._logger.info(f"\t\t\tE [{1+self._episode:0>{len(str(episode_limit))}}/{episode_limit}]")

    @property
    def contest_index(self) -> Contest.Index:
        return self._contest_index
    
    @contest_index.setter
    def contest_index(self, index: int):
        self._contest_index = Contest.Index(index)

        if self._contest_index > self._contest.max_index:
            self._logger.info("BENCHMARK COMPLETED SUCCESSFULLY")
            os.remove(self.DIR(self.LOCK_FILE))
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
                self._logger.debug(f"map change requires restart")
                self._requires_restart = True
                
            if new_config.robot != old_config.robot:
                self._logger.debug(f"robot change requires restart")
                self._requires_restart = True
            
            self._reincarnate()

    @property
    def _episode(self) -> int:
        return self._episode_index
    
    @_episode.setter
    def _episode(self, episode: int):
        if episode >= int(self._suite.config(self._suite_index).episodes * self._config.suite.scale_episodes):
            self._episode_index = 0
            self.suite_index += 1
        else:
            self._episode_index = episode
            self._log_episode()

    def _reincarnate(self):

        with open(self.DIR(self.LOCK_FILE), "w") as f:
            f.write(f"{self._runid} {self._contest_index} {self._suite_index} {self._headless}")

        config = self._config
        contest_config = self._contest.config(self._contest_index)
        suite_config = self._suite.config(self._suite_index)

        self._taskgen_restore()
        self._taskgen_write(
            {
                "episodes": -1,
                "RANDOM": {
                    "seed": suite_config.seed ^ Suite.Stage.hash({"":self._runid})
                }
            },
            suite_config.config
        )

        record_data_dir = f"{self._runid}/{contest_config.name}/{suite_config.name}"

        if self._requires_restart:
            self._logger.info(f"{_get_rosmaster_pid()}")
            subprocess.Popen(
                [
                    os.path.join(
                        rospkg.RosPack().get_path("task_generator"),
                        "scripts",
                        "delay_restart.py"
                    ),
                    f"{_get_rosmaster_pid()}",
                    "arena_bringup",
                    "start_arena.launch",
                    "tm_modules:=benchmark",
                    "benchmark_resume:=true",
                    "record_data:=true",
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
                    f"tm_obstacles:={suite_config.tm_obstacles}"
                ],
                start_new_session=True
            )
            self._suicide()
        
        else:
            rospy.ServiceProxy(f"/{suite_config.robot}/change_directory", arena_evaluation_srvs.ChangeDirectory).call(arena_evaluation_srvs.ChangeDirectoryRequest(record_data_dir))
            self._episode = 0

    def _suicide(self):
        subprocess.run(["kill", f"{_get_rosmaster_pid()}"])
        rospy.signal_shutdown("goodbye cruel world")