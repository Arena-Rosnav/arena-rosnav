import copy
import os
import re
import tempfile
import typing

import yaml
import launch
import launch.actions
import launch.substitutions
import launch.conditions
import launch.utilities


def _yaml_iter(obj: list | dict):
    if isinstance(obj, dict):
        return obj.keys()
    elif isinstance(obj, list):
        return range(len(obj))
    else:
        raise RuntimeError()


class LaunchArgument(launch.actions.DeclareLaunchArgument):
    @property
    def substitution(self):
        return launch.substitutions.LaunchConfiguration(self.name)

    @property
    def parameter(self):
        return {self.name: self.substitution}


class SelectAction(launch.Action):
    _actions: dict[str, list[launch.Action]]
    _selector: launch.SomeSubstitutionsType

    def __init__(
        self,
        selector: launch.SomeSubstitutionsType
    ) -> None:
        launch.Action.__init__(self)
        self._actions = {}
        self._selector = selector

    def add(self, value: str, action: launch.Action):
        self._actions.setdefault(value, []).append(
            action
        )

    def execute(self, context: launch.LaunchContext):
        key = launch.utilities.perform_substitutions(
            context, (self._selector,))
        return self._actions.get(key, [])


class YAMLFileSubstitution(launch.Substitution):

    _path: launch.SomeSubstitutionsType
    _default: typing.Optional[dict | typing.Self]
    _substitute: bool

    def __init__(
        self,
        path: launch.SomeSubstitutionsType,
        *,
        default: typing.Optional[dict | typing.Self] = None,
        substitute: bool = False,
    ):
        launch.Substitution.__init__(self)
        self._path = path
        self._default = default
        self._substitute = substitute

    def perform(
        self,
        context: launch.LaunchContext,
    ):
        yaml_path = launch.utilities.perform_substitutions(context, self._path)
        try:
            with open(yaml_path) as f:
                contents = yaml.safe_load(f)
        except BaseException as e:
            if self._default is not None:
                if isinstance(self._default, type(self)):
                    contents = self._default.perform_load(context)
                else:
                    contents = self._default
            else:
                raise e

        if self._substitute:
            def substitute(obj, k):
                if isinstance(obj[k], launch.Substitution):
                    obj[k] = obj[k].perform(context)

            def substitute_recursive(obj: dict | list):
                for k in _yaml_iter(obj):
                    substitute(obj, k)
                    v = obj[k]
                    if isinstance(v, (list, dict)):
                        obj[k] = substitute_recursive(v)

                return obj

            contents = substitute_recursive(contents)

        yaml_file = tempfile.NamedTemporaryFile(mode='w', delete=False)
        yaml.safe_dump(contents, yaml_file)
        yaml_file.close()

        return yaml_file.name

    def perform_load(self, context: launch.LaunchContext) -> dict:
        with open(yaml_path := self.perform(context)) as f:
            content = yaml.safe_load(f)
        if not isinstance(content, dict):
            raise yaml.YAMLError(
                f"{yaml_path} does not contain a top-level dictionary")
        return content

    @classmethod
    def from_dict(cls, obj: dict, /, *, substitute: bool = False):
        return cls(path=[], default=obj, substitute=substitute)


class YAMLMergeSubstitution(launch.Substitution):

    _base: YAMLFileSubstitution
    _yamls: typing.Iterable[YAMLFileSubstitution]

    def __init__(
        self,
        base: YAMLFileSubstitution,
        *yamls: YAMLFileSubstitution,
    ):
        launch.Substitution.__init__(self)
        self._base = base
        self._yamls = yamls

    @classmethod
    def _recursive_merge(cls, obj1: dict, obj2: dict) -> dict:
        for k, v in obj2.items():
            if k in obj1 and isinstance(v, dict):
                v = cls._recursive_merge(obj1[k], v)
            obj1[k] = v
        return obj1

    @classmethod
    def _append_yaml(cls, base: dict, obj: dict) -> dict:
        base = cls._recursive_merge(base, obj)
        return base

    def perform(self, context: launch.LaunchContext):

        combined = copy.deepcopy(self._base.perform_load(context))

        for yaml_path in self._yamls:
            combined = self._append_yaml(
                combined,
                yaml_path.perform_load(context)
            )

        return YAMLFileSubstitution.from_dict(combined).perform(context)


class YAMLReplaceSubstitution(launch.Substitution):

    _substitutions: YAMLFileSubstitution
    _obj: YAMLFileSubstitution

    def __init__(
        self,
        *,
        substitutions: YAMLFileSubstitution,
        obj: YAMLFileSubstitution,
    ):
        launch.Substitution.__init__(self)
        self._substitutions = substitutions
        self._obj = obj

    def perform(self, context: launch.LaunchContext):

        substitutions = self._substitutions.perform_load(context)

        def recurse(obj: list | dict):
            for k in _yaml_iter(obj):
                if isinstance(obj[k], str) and \
                        (match := re.match(r'\$\{([^\}]*)\}', obj[k])) is not None:

                    sub = match.group(1)
                    if sub in substitutions:
                        obj[k] = substitutions[sub]

                if isinstance(obj[k], (dict, list)):
                    obj[k] = recurse(obj[k])

            return obj

        result = recurse(self._obj.perform_load(context))
        assert isinstance(result, dict)

        return YAMLFileSubstitution.from_dict(
            result,
            substitute=True,
        ).perform(context)
