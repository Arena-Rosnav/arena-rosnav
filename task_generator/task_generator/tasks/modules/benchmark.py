from task_generator.constants import Constants
from task_generator.shared import Namespace
from task_generator.tasks.modules import TM_Module
from task_generator.tasks.task_factory import TaskFactory

import typing
import dataclasses
import os
import yaml
import subprocess

import rospkg
import rospy

def _get_rosmaster_pid() -> int:
    try:
        return int(subprocess.check_output(["ps", "-C", "rosmaster", "-o", "pid", "h"]).decode())
    except Exception as e:
        raise RuntimeError("could not determine rosmaster pid") from e

class Config(typing.NamedTuple):

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
    def parse(cls, obj: typing.Dict):
        return cls(
            stages = [
                cls.Stage(**stage)
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
    def parse(cls, obj: typing.Dict):
        return cls(
            contestants = [
                cls.Contestant(**contestant)
                for contestant
                in obj["contestants"]
            ]
        )

    class Index(int): ...

    class Contestant(typing.NamedTuple):
        name: str
        local_planner: str
        inter_planner: str

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
    TASK_GENERATOR_CONFIG = os.path.join(
        rospkg.RosPack().get_path("arena_bringup"),
        "configs",
        "task_generator.yaml"
    )
    TASK_GENERATOR_CONFIG_BKUP = TASK_GENERATOR_CONFIG + ".bkup"

    _config: Config
    _suite: Suite
    _contest: Contest
    _episode_index: int

    _contest_index: Contest.Index
    _suite_index: Suite.Index

    # CONFIGURATION

    @classmethod
    def _load_config(cls) -> Config:
        with open(cls.DIR("config.yaml")) as f:
            return Config.parse(yaml.load(f, yaml.FullLoader))

    @classmethod
    def _load_contest(cls, contest: str) -> Contest:
        with open(cls.DIR("contests", contest)) as f:
            return Contest.parse(yaml.load(f, yaml.FullLoader))
        
    @classmethod
    def _load_suite(cls, suite: str) -> Suite:
        with open(cls.DIR("suites", suite)) as f:
            return Suite.parse(yaml.load(f, yaml.FullLoader))
    
    @classmethod
    def _resume(cls) -> typing.Tuple[Contest.Index, Suite.Index]:
        with open(cls.DIR(cls.LOCK_FILE)) as f:
            contest, suite = f.read().split(" ")
        return Contest.Index(contest), Suite.Index(suite)


    @classmethod
    def _taskgen_backup(cls):
        with open(cls.TASK_GENERATOR_CONFIG) as fr:
            with open(cls.TASK_GENERATOR_CONFIG + ".bkup", "w") as fw:
                fw.write(fr.read())

    @classmethod
    def _taskgen_write(cls, config: typing.Dict):

        def overwrite(source: typing.Dict, target: typing.Dict):
            for k,v in source.items():
                if isinstance(v, dict):
                    target.setdefault(k, dict())
                    overwrite(v, target[k])
                else:
                    target[k] = v
            return target

        with open(cls.TASK_GENERATOR_CONFIG, "r") as f:
            config = overwrite(config, yaml.load(f, yaml.FullLoader))
        
        # yaml has problems with r+
        with open(cls.TASK_GENERATOR_CONFIG, "w") as f:
            yaml.dump(config, f)

    @classmethod
    def _taskgen_restore(cls):
        with open(cls.TASK_GENERATOR_CONFIG_BKUP) as fr:
            with open(cls.TASK_GENERATOR_CONFIG, "w") as fw:
                fw.write(fr.read())
        

    # RUNTIME

    def __init__(self, **kwargs):
        
        self._config = self._load_config()
        self._suite = self._load_suite(self._config.suite.config)
        self._contest = self._load_contest(self._config.contest.config)
        self._episode_index = 0

        # first run
        if not rospy.get_param("/benchmark_resume", False):
            self._taskgen_backup()
            self._contest_index = self._contest.min_index
            self._suite_index = self._suite.min_index
            return self._reincarnate()

        self._contest_index, self._suite_index = self._resume()


    def before_reset(self):
        pass
    
    def after_reset(self):
        self._episode += 1

    @property
    def contest_index(self) -> Contest.Index:
        return self._contest_index
    
    @contest_index.setter
    def contest_index(self, index: int):
        self._contest_index = Contest.Index(index)

        if self._contest_index > self._contest.max_index:
            os.remove(self.DIR(self.LOCK_FILE))
            self._taskgen_restore()
            self._suicide()
        else:
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
            self._reincarnate()

    @property
    def _episode(self) -> int:
        return self._episode_index
    
    @_episode.setter
    def _episode(self, episode: int):
        self._episode_index = episode

        if self._episode_index > int(self._suite.config(self._suite_index).episodes * self._config.suite.scale_episodes):
            self._episode_index = 0
            self.suite_index += 1


    def _reincarnate(self):

        with open(self.DIR(self.LOCK_FILE), "w") as f:
            f.write(f"{self._contest_index} {self._suite_index}")

        config = self._config
        contest_config = self._contest.config(self._contest_index)
        suite_config = self._suite.config(self._suite_index)

        self._taskgen_restore()
        self._taskgen_write(suite_config.config)

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

                f"simulator:={config.general.simulator}",
                
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

    def _suicide(self):
        subprocess.run(["kill", f"{_get_rosmaster_pid()}"])
        rospy.signal_shutdown("goodbye cruel world")