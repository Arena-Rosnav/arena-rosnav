import abc
import functools
import json
import os
import re
import subprocess
import sys
import tempfile
import typing
import xml.etree.ElementTree as ET
from typing import Collection, Optional, Set, Type

import attrs

import task_generator.utils.arena as Utils
from task_generator.shared import Model, ModelType, ModelWrapper


class _ModelLoader(abc.ABC):
    @classmethod
    def load(cls, model_dir: str, model: str, loader_args: dict) -> Model | None:
        return None

    @classmethod
    def convertable(cls) -> Collection[ModelType]:
        """
        return collection of model types convertable
        """
        return ()

    @classmethod
    def convert(cls, model_dir: str, model: Model, loader_args: dict) -> Model | None:
        return None


class ModelLoader:

    _registry: dict[ModelType, Type[_ModelLoader]] = {}
    _models: Set[str]

    @classmethod
    def model(cls, model_type: ModelType):
        def inner(loader: Type[_ModelLoader]):
            cls._registry[model_type] = loader
        return inner

    _model_dir: str
    _cache: dict[tuple[ModelType, str], Model]

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
        return ModelWrapper(
            name=model,
            callback=functools.partial(self._load_safe, model)
        )

    def _load(self, model: str, only: Collection[ModelType], loader_args: dict) -> Model | None:
        if not only:
            only = self._registry.keys()

        only = [t for t in only if t in self._registry]

        for model_type in only:  # cache pass
            if (model_type, model) in self._cache:
                return self._cache[(model_type, model)]

        for model_type in only:  # disk pass
            hit = self._registry[model_type].load(self._model_dir, model, loader_args)
            if hit is not None:
                self._cache[(model_type, model)] = hit
                return self._cache[(model_type, model)]

        for model_type in only:  # try to convert
            targets = self._registry[model_type].convertable()
            if not targets:
                continue
            match = self._load(model, targets, loader_args)
            if match is not None:
                converted = self._registry[model_type].convert(self._model_dir, match, loader_args)
                if converted is None:
                    continue

                self._cache[(model_type, model)] = converted
                return converted

        return None

    def _load_safe(self, model: str, only: Collection[ModelType], loader_args: dict) -> Model:
        loaded = self._load(model, only, loader_args)
        if loaded is not None:
            return loaded

        raise FileNotFoundError(f"no model {model} among {only} found in {self._model_dir} and could not be converted")


@ModelLoader.model(ModelType.YAML)
class _ModelLoader_YAML(_ModelLoader):

    @classmethod
    def load(cls, model_dir, model, loader_args):

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
    def load(cls, model_dir, model, loader_args):
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
    def load(cls, model_dir, model, loader_args):

        namespace: Optional[str] = loader_args.get("namespace", None)

        base_path = os.path.join(model_dir, model, "urdf")
        xacro_path = os.path.join(base_path, f"{model}.urdf.xacro")
        model_path = os.path.join(base_path, f"{model}.urdf")

        if not os.path.isfile(xacro_path):
            return None

        try:
            def to_string(v: typing.Any) -> str:
                if attrs.has(type(v)):
                    v = attrs.asdict(v)
                if isinstance(v, dict):
                    return json.dumps(v)
                return str(v)

            model_desc = subprocess.check_output([
                "ros2",
                "run",
                "xacro",
                "xacro",
                xacro_path,
                *(
                    f"{k}:={to_string(v)}"
                    for k, v
                    in loader_args.items()
                ),
            ]).decode("utf-8")

            with open(model_path, 'w') as f:
                f.write(model_desc)

            base_dir = os.path.dirname(model_path)
            tree = ET.parse(model_path)
            root = tree.getroot()

            prefix = "package://jackal_description"

            # Iterate over every element in the XML tree and update 'filename' attributes
            for elem in root.iter():
                if 'filename' in elem.attrib:
                    original_path = elem.attrib['filename']
                    # Remove the specific package prefix if present
                    if original_path.startswith(prefix):
                        # Remove the prefix and any leading '/'
                        new_relative = original_path[len(prefix):].lstrip('/')
                        original_path = new_relative
                        print(f"Removed prefix: {prefix} -> New relative path: {original_path}")
                    # Convert to absolute path if it's not already
                    if not os.path.isabs(original_path):
                        abs_path = os.path.abspath(os.path.join(base_dir, original_path))
                        elem.attrib['filename'] = abs_path
                        print(f"Updated relative path to absolute: {original_path} -> {abs_path}")

            # Write the updated XML tree to a temporary file
            with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf", mode="wb") as tmp:
                tree.write(tmp, encoding="utf-8", xml_declaration=True)
                temp_filepath = tmp.name
                print(f"Converted URDF saved to temporary file: {temp_filepath}")

            return Model(
                type=ModelType.URDF,
                name=model,
                description=model_desc,
                path=temp_filepath
            )

        except subprocess.CalledProcessError as e:
            print(
                f"error processing model {model} URDF file {model_path}. refusing to load.\n{e}\n{e.output.decode('utf-8')}",
                file=sys.stderr
            )
            return None


@ModelLoader.model(ModelType.USD)
class _ModelLoader_USD(_ModelLoader):
    @classmethod
    def load(cls, model_dir, model, loader_args):
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
    def convert(cls, model_dir: str, model: Model, loader_args) -> Model | None:
        if model.type == ModelType.SDF:
            try:
                # print(model_dir)
                sdf_model_path = os.path.join(model_dir, model.name, "sdf", f"{model.name}.sdf")
                materials_path = os.path.join(model_dir, model.name, 'sdf', 'materials')
                tree = ET.parse(sdf_model_path)
                root = tree.getroot()
                # First pass: resolve package:// URIs
                model_uri_pattern = re.compile(r'^model://([^/]+)(.*)$')
                package_uri_pattern = re.compile(r'^package://([^/]+)(.*)$')
                for uri_elem in root.iter():
                    if uri_elem.text:
                        text = uri_elem.text.strip()
                        match = model_uri_pattern.match(text)
                        if match:
                            package_name = match.group(1)
                            remaining_path = match.group(2)
                            # Get the absolute path for the package share directory.
                            # Replace the package URI with the resolved directory plus remaining path.
                            new_uri = model_dir + '/' + model.name + remaining_path
                            print(new_uri)
                            # uri_elem.text = new_uri
                            if (new_uri.endswith('.dae') or new_uri.endswith('.DAE')) and os.path.exists(new_uri):
                                new_dae_path = Utils.process_dae(new_uri, materials_path)
                                uri_elem.text = new_dae_path
                            elif new_uri.endswith('.obj') and os.path.exists(new_uri):
                                new_obj_path = Utils.process_obj(new_uri, materials_path)
                                uri_elem.text = new_obj_path
                        else:
                            match = package_uri_pattern.match(text)
                            if match:
                                package_name = match.group(1)
                                remaining_path = match.group(2)
                                # Get the absolute path for the package share directory.
                                # Replace the package URI with the resolved directory plus remaining path.
                                new_uri = model_dir + '/' + model.name + remaining_path
                                print(new_uri)
                                if (new_uri.endswith('.dae') or new_uri.endswith('.DAE')) and os.path.exists(new_uri):
                                    new_dae_path = Utils.process_dae(new_uri, materials_path)
                                    uri_elem.text = new_dae_path
                                elif new_uri.endswith('.obj') and os.path.exists(new_uri):
                                    new_obj_path = Utils.process_obj(new_uri, materials_path)
                                    uri_elem.text = new_obj_path
                model_path = os.path.join(model_dir, model.name, "usd", f"{model.name}.usd")
                os.makedirs(os.path.dirname(model_path), exist_ok=True)
                if os.path.islink(model_path) and not os.path.exists(model_path):  # broken symlink
                    os.unlink(model_path)

                ARENA_WS_DIR = Utils.get_arena_ws()

                env = os.environ.copy()
                env['ARENA_WS_DIR'] = ARENA_WS_DIR
                with tempfile.NamedTemporaryFile(delete=False, mode='w') as f:
                    tree.write(f, encoding='unicode')
                    f.flush()
                    temp_file_path = f.name
                    print("Temporary SDF file for converter:", temp_file_path)
                    subprocess.check_output(

                        [
                            f'{ARENA_WS_DIR}/src/arena/arena-rosnav/tools/sdf2usd',
                            f.name,
                            model_path
                        ],
                        env=env,
                        # shell=True,
                    )

                from pxr import Usd
                stage = Usd.Stage.Open(model_path)
                for prim in stage.Traverse():
                    if prim.GetTypeName() == "Xform":
                        first_xform_prim = prim
                        break
                prim_path = first_xform_prim.GetPath()
                # print(prim_path)
                stage.SetDefaultPrim(first_xform_prim)
                root_layer = stage.GetRootLayer()
                root_layer.Save()
                return cls.load(model_dir, model.name, loader_args)

            except Exception:
                return None

        return None
