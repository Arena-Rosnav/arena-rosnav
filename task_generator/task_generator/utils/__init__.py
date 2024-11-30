import functools
import subprocess
from typing import Callable, Collection, Dict, Iterator, List, Optional, Set, Tuple, Type

import os

import heapq
import itertools

from task_generator.shared import ModelWrapper, Model, ModelType


class NamespaceIndexer:

    _freed: List[int]
    _gen: Iterator[int]
    _namespace: str
    _sep: str

    def __init__(self, namespace: str, sep: str = "_"):
        self._freed = list()
        self._gen = itertools.count()
        self._namespace = namespace
        self._sep = sep

    def free(self, index: int):
        heapq.heappush(self._freed, index)

    def get(self) -> int:
        if len(self._freed):
            return heapq.heappop(self._freed)

        return next(self._gen)

    def format(self, index: int) -> str:
        return f"{self._namespace}{self._sep}{index}"

    def __next__(self) -> Tuple[str, Callable[[], None]]:
        index = self.get()
        return self.format(index), lambda: self.free(index)


class _ModelLoader:
    @staticmethod
    def load(model_dir: str, model: str, **kwargs) -> Optional[Model]:
        ...


class ModelLoader:

    _registry: Dict[ModelType, Type[_ModelLoader]] = {}
    _models: Set[str]

    @classmethod
    def model(cls, model_type: ModelType):
        def inner(loader: Type[_ModelLoader]):
            cls._registry[model_type] = loader
        return inner

    _model_dir: str

    def __init__(self, model_dir: str):
        self._model_dir = model_dir
        self._cache = dict()
        self._models = set()

        # # potentially expensive
        # rospy.logdebug(
        #     f"models in {os.path.basename(model_dir)}: {self.models}")

    @property
    def models(self) -> Set[str]:
        if not len(self._models):
            if os.path.isdir(self._model_dir):
                self._models = set(next(os.walk(self._model_dir))[1])
            else:
                # rospy.logwarn(
                # f"Model directory {self._model_dir} does not exist. No models
                # are provided.")
                self._models = set()

        return self._models

    def bind(self, model: str) -> ModelWrapper:
        return ModelWrapper.bind(
            name=model, callback=functools.partial(self._load, model))

    def _load(self, model: str,
              only: Collection[ModelType], **kwargs) -> Model:

        if not len(only):
            only = self._registry.keys()

        for model_type in only:  # cache pass
            if (model_type, model) in self._cache:
                return self._cache[(model_type, model)]

        for model_type in only:  # disk pass
            hit = self._load_single(
                model_type=model_type, model=model, **kwargs)
            if hit is not None:
                self._cache[(model_type, model)] = hit
                return self._cache[(model_type, model)]

        else:
            # TODO refactor so None is returned instead of raising error
            raise FileNotFoundError(
                f"no model {model} among {only} found in {self._model_dir}")

    def _load_single(self, model_type: ModelType, model: str,
                     **kwargs) -> Optional[Model]:
        if model_type in self._registry:
            return self._registry[model_type].load(
                self._model_dir, model, **kwargs)

        return None

    def getArenaDir():
        current_dir = os.path.abspath(__file__)
        workspace_root = current_dir

        while not workspace_root.endswith('arena4_ws'):
            workspace_root = os.path.dirname(workspace_root)

        if not workspace_root.endswith('arena4_ws'):
            raise ValueError(
                "Could not find the 'arena4_ws' directory in the current path.")

        return workspace_root


@ModelLoader.model(ModelType.YAML)
class _ModelLoader_YAML(_ModelLoader):

    @staticmethod
    def load(model_dir, model, **kwargs):

        model_path = os.path.join(model_dir, model, "yaml", f"{model}.yaml")

        try:
            with open(model_path) as f:
                model_desc = f.read()
        except FileNotFoundError:
            return None

        else:
            model_obj = Model(
                type=ModelType.YAML,
                name=model,
                description=model_desc,
                path=model_path
            )
            return model_obj


@ModelLoader.model(ModelType.SDF)
class _ModelLoader_SDF(_ModelLoader):

    @staticmethod
    def load(model_dir, model, **kwargs):

        model_path = os.path.join(model_dir, model, "sdf", f"{model}.sdf")

        try:
            with open(model_path) as f:
                model_desc = f.read()
        except FileNotFoundError:
            return None

        else:
            model_obj = Model(
                type=ModelType.SDF,
                name=model,
                description=model_desc,
                path=model_path
            )
            return model_obj


@ModelLoader.model(ModelType.URDF)
class _ModelLoader_URDF(_ModelLoader):

    @staticmethod
    def load(model_dir, model, **kwargs):

        namespace: Optional[str] = kwargs.get("namespace", None)

        model_path = os.path.join(
            model_dir, model, "urdf", f"{model}.urdf.xacro")

        if not os.path.isfile(model_path):
            return None

        try:
            model_desc = subprocess.check_output([
                "rosrun",
                "xacro",
                "xacro",
                model_path,
                *([f"""robot_namespace:={namespace}"""] if namespace is not None else [])
            ]).decode("utf-8")

        except subprocess.CalledProcessError as e:
            # rospy.logerr_once(
            # f"error processing model {model} URDF file {model_path}. refusing
            # to load.\n{e}\n{e.output.decode('utf-8')}")
            return None

        else:
            model_obj = Model(
                type=ModelType.URDF,
                name=model,
                description=model_desc,
                path=model_path
            )
            return model_obj
