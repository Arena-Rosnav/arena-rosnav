import functools
import subprocess
from typing import Callable, Collection, Dict, Iterator, List, Optional, Set, Tuple, Type, TypeVar


import rospy
import os
import numpy as np

import heapq
import itertools

from task_generator.constants import Constants
from task_generator.shared import ModelWrapper, Model, ModelType

import nav_msgs.msg as nav_msgs


class Utils:
    @staticmethod
    def get_simulator() -> Constants.Simulator:
        return Constants.Simulator(rosparam_get(str, "simulator", "flatland").lower())

    @staticmethod
    def get_arena_type() -> Constants.ArenaType:
        return Constants.ArenaType(os.getenv("ARENA_TYPE", "training").lower())

    @staticmethod
    def is_synthetic_map() -> bool:
        return rosparam_get(str, "map_file") in ["dynamic_map"]

    @staticmethod
    def generate_map_inner_border(free_space_indices, map_: nav_msgs.OccupancyGrid):
        """generate map border (four vertices of the map)

        Returns:
            vertex_coordinate_x_y(np.ndarray with shape 4 x 2):
        """
        n_freespace_cells = len(free_space_indices[0])
        border_vertex = np.array([]).reshape(0, 2)
        border_vertices = np.array([]).reshape(0, 2)
        for idx in [0, n_freespace_cells - 4]:
            y_in_cells, x_in_cells = free_space_indices[0][idx], free_space_indices[1][idx]
            y_in_meters = y_in_cells * map_.info.resolution + map_.info.origin.position.y
            x_in_meters = x_in_cells * map_.info.resolution + map_.info.origin.position.x
            border_vertex = np.vstack(
                [border_vertex, [x_in_meters, y_in_meters]])
        border_vertices = np.vstack(
            [border_vertices, [border_vertex[0, 0], border_vertex[0, 1]]])
        border_vertices = np.vstack(
            [border_vertices, [border_vertex[0, 0], border_vertex[1, 1]]])
        border_vertices = np.vstack(
            [border_vertices, [border_vertex[1, 0], border_vertex[1, 1]]])
        border_vertices = np.vstack(
            [border_vertices, [border_vertex[1, 0], border_vertex[0, 1]]])
        # print('border',border_vertices)
        return border_vertices

    @staticmethod
    def update_freespace_indices_maze(map_: nav_msgs.OccupancyGrid):
        """update the indices(represented in a tuple) of the freespace based on the map and the static polygons
        ostacles manuelly added 
        param map_ : original occupacy grid
        param vertlist: vertex of the polygons

        Returns:
            indices_y_x(tuple): indices of the non-occupied cells, the first element is the y-axis indices,
            the second element is the x-axis indices.
        """
        width_in_cell, height_in_cell = map_.info.width, map_.info.height
        map_2d = np.reshape(map_.data, (height_in_cell, width_in_cell))
        # height range and width range
        wall_occupancy = np.array([[1.25, 12.65, 10.6, 10.8],
                                   [-4.45, 18.35, 16.3, 16.5],
                                  [-4.45, 18.35, 4.9, 5.1],
                                   [12.55, 12.75, -0.7, 22.1],
                                   [1.15, 1.35, -0.7, 22.1],
                                   [6.85, 7.05, 5.0, 16.4]])
        size = wall_occupancy.shape[0]
        for ranges in wall_occupancy:
            height_low = int(ranges[0] / map_.info.resolution)
            height_high = int(ranges[1] / map_.info.resolution)
            width_low = int(ranges[2] / map_.info.resolution)
            width_high = int(ranges[3] / map_.info.resolution)
            height_grid = height_high - height_low
            width_grid = width_high - width_low
            for i in range(height_grid):
                y = height_low + i
                for j in range(width_grid):
                    x = width_low + j
                    map_2d[y, x] = 100
        free_space_indices_new = np.where(map_2d == 0)
        return free_space_indices_new


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

        # potentially expensive
        rospy.logdebug(
            f"models in {os.path.basename(model_dir)}: {self.models}")

    @property
    def models(self) -> Set[str]:
        if not len(self._models):
            if os.path.isdir(self._model_dir):
                self._models = set(next(os.walk(self._model_dir))[1])
            else:
                rospy.logwarn(
                    f"Model directory {self._model_dir} does not exist. No models are provided.")
                self._models = set()

        return self._models

    def bind(self, model: str) -> ModelWrapper:
        return ModelWrapper.bind(name=model, callback=functools.partial(self._load, model))

    def _load(self, model: str, only: Collection[ModelType], **kwargs) -> Model:

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

    def _load_single(self, model_type: ModelType, model: str, **kwargs) -> Optional[Model]:
        if model_type in self._registry:
            return self._registry[model_type].load(self._model_dir, model, **kwargs)

        return None


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
            rospy.logerr_once(
                f"error processing model {model} URDF file {model_path}. refusing to load.\n{e}\n{e.output.decode('utf-8')}")
            return None

        else:
            model_obj = Model(
                type=ModelType.URDF,
                name=model,
                description=model_desc,
                path=model_path
            )
            return model_obj


T = TypeVar("T")
_unspecified = rospy.client._Unspecified()


def rosparam_get(cast: Type[T], param_name: str, default=_unspecified, strict: bool = False) -> T:
    val = rospy.get_param(param_name=param_name, default=default)

    if strict:
        if not isinstance(val, cast):
            raise ValueError(
                f"param {param_name} is not of type {cast} but {type(val)} with val {val}")
    else:
        try:
            val = cast(val)
        except ValueError as e:
            raise ValueError(f"could not cast {val} to {cast}", e)

    return val
