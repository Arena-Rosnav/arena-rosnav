import abc
import functools
import os
import subprocess
import tempfile
from typing import Collection, Dict, Optional, Set, Tuple, Type

from task_generator.shared import Model, ModelType, ModelWrapper
import task_generator.utils.arena as Utils


class _ModelLoader(abc.ABC):
    @classmethod
    def load(cls, model_dir: str, model: str, **kwargs) -> Model | None:
        return None

    @classmethod
    def convertable(cls) -> Collection[ModelType]:
        """
        return collection of model types convertable
        """
        return ()

    @classmethod
    def convert(cls, model_dir: str, model: Model, **kwargs) -> Model | None:
        return None


class ModelLoader:

    _registry: Dict[ModelType, Type[_ModelLoader]] = {}
    _models: Set[str]

    @classmethod
    def model(cls, model_type: ModelType):
        def inner(loader: Type[_ModelLoader]):
            cls._registry[model_type] = loader
        return inner

    _model_dir: str
    _cache: Dict[Tuple[ModelType, str], Model]

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
            name=model, callback=functools.partial(self._load_safe, model))

    def _load(self, model: str, only: Collection[ModelType], **kwargs) -> Model | None:
        if not only:
            only = self._registry.keys()

        only = [t for t in only if t in self._registry]

        for model_type in only:  # cache pass
            if (model_type, model) in self._cache:
                return self._cache[(model_type, model)]

        for model_type in only:  # disk pass
            hit = self._registry[model_type].load(model_dir=self._model_dir, model=model, **kwargs)
            if hit is not None:
                self._cache[(model_type, model)] = hit
                return self._cache[(model_type, model)]

        for model_type in only:  # try to convert
            match = self._load(model, only=self._registry[model_type].convertable())
            if match is not None:
                converted = self._registry[model_type].convert(self._model_dir, match, **kwargs)
                if converted is None:
                    continue

                self._cache[(model_type, model)] = converted
                return converted

        return None

    def _load_safe(self, model: str, only: Collection[ModelType], **kwargs) -> Model:
        loaded = self._load(model, only, **kwargs)
        if loaded is not None:
            return loaded

        raise FileNotFoundError(f"no model {model} among {only} found in {self._model_dir} and could not be converted")


@ModelLoader.model(ModelType.YAML)
class _ModelLoader_YAML(_ModelLoader):

    @classmethod
    def load(cls, model_dir, model, **kwargs):

        model_path = os.path.join(model_dir, model, "yaml", f"{model}.yaml")

        try:
            with open(model_path) as f:
                model_desc = f.read()
        except FileNotFoundError:
            return None

        model_obj = Model(
            type=ModelType.YAML,
            name=model,
            description=model_desc,
            path=model_path
        )
        return model_obj


@ModelLoader.model(ModelType.SDF)
class _ModelLoader_SDF(_ModelLoader):

    @classmethod
    def load(cls, model_dir, model, **kwargs):
        model_path = os.path.join(model_dir, model, "sdf", f"{model}.sdf")
        try:
            with open(model_path) as f:
                return Model(
                    type=ModelType.SDF,
                    name=model,
                    description=f.read(),
                    path=model_path
                )
        except FileNotFoundError:
            pass
        return None


@ModelLoader.model(ModelType.URDF)
class _ModelLoader_URDF(_ModelLoader):

    @classmethod
    def load(cls, model_dir, model, **kwargs):

        namespace: Optional[str] = kwargs.get("namespace", None)

        model_path = os.path.join(model_dir, model, "urdf", f"{model}.urdf.xacro")

        if not os.path.isfile(model_path):
            return None

        try:
            model_desc = subprocess.check_output([
                "ros2",
                "run",
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


@ModelLoader.model(ModelType.USD)
class _ModelLoader_USD(_ModelLoader):
    @classmethod
    def load(cls, model_dir, model, **kwargs):
        model_path = os.path.join(model_dir, model, "usd", f"{model}.usd")
        try:
            with open(model_path, 'rb') as f:
                return Model(
                    type=ModelType.USD,
                    name=model,
                    description="",  # TODO add bytes compat
                    path=model_path
                )
        except FileNotFoundError:
            pass
        return None

    @classmethod
    def convertable(cls) -> Collection[ModelType]:
        return (ModelType.SDF,)

    @classmethod
    def convert(cls, model_dir: str, model: Model, **kwargs) -> Model | None:
        if model.type == ModelType.SDF:
            try:
                model_path = os.path.join(model_dir, model.name, "usd", f"{model.name}.usd")
                os.makedirs(os.path.dirname(model_path), exist_ok=True)

                ARENA_WS_DIR = Utils.get_arena_ws()

                env = os.environ.copy()
                env['ARENA_WS_DIR'] = ARENA_WS_DIR
                with tempfile.NamedTemporaryFile('w') as f:
                    f.write(model.description)
                    f.flush()
                    subprocess.check_output(
                        [
                            f'{ARENA_WS_DIR}/src/arena/arena-rosnav/tools/sdf2usd',
                            f.name,
                            model_path
                        ],
                        env=env,
                        # shell=True,
                    )

                return cls.load(model_dir, model.name, **kwargs)

            except Exception:
                return None

        return None
