import copy
import dataclasses
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
import launch_ros


class NoAliasDumper(yaml.Dumper):
    def ignore_aliases(self, data):
        return True


def _yaml_iter(obj: list | dict):
    if isinstance(obj, dict):
        return obj.keys()
    elif isinstance(obj, list):
        return range(len(obj))
    else:
        raise RuntimeError()


class LaunchArgument(launch.actions.DeclareLaunchArgument):

    _auto_append: typing.ClassVar[typing.List | None] = None

    @classmethod
    def auto_append(cls, target: typing.List | None = None):
        """
        auto append self to list after creation
        """
        cls._auto_append = target

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if self._auto_append is not None:
            self._auto_append.append(self)

    @property
    def substitution(self):
        return launch.substitutions.LaunchConfiguration(self.name)

    @property
    def dict(self):
        return {self.name: self.substitution}

    def param_value(self, type_: typing.Type):
        return launch_ros.parameter_descriptions.ParameterValue(self.substitution, value_type=type_)

    def param(self, type_: typing.Type):
        return {self.name: self.param_value(type_)}

    @property
    def str_param(self):
        return self.param(str)


class SelectAction(launch.Action):
    _actions: dict[str, list[launch.Action]]
    _selector: list[launch.Substitution]

    def __init__(
        self,
        selector: launch.SomeSubstitutionsType
    ) -> None:
        launch.Action.__init__(self)
        self._actions = {}
        self._selector = launch.utilities.normalize_to_list_of_substitutions(selector)

    def add(self, value: str, action: launch.Action):
        self._actions.setdefault(value, []).append(
            action
        )

    def execute(self, context: launch.LaunchContext):
        key = launch.utilities.perform_substitutions(context, self._selector)
        return self._actions.get(key, [])


class YAMLFileSubstitution(launch.Substitution):

    _path: list[launch.Substitution]
    _default: "typing.Optional[dict | YAMLFileSubstitution]"
    _substitute: bool

    def __init__(
        self,
        path: launch.SomeSubstitutionsType,
        *,
        default: "typing.Optional[dict | YAMLFileSubstitution]" = None,
        substitute: bool = False,
    ):
        launch.Substitution.__init__(self)
        self._path = launch.utilities.normalize_to_list_of_substitutions(path)
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
        yaml.dump(contents, yaml_file, Dumper=NoAliasDumper)
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
    def from_dict(cls, obj: dict, /, *, substitute: bool = True):
        return cls(path=[], default=obj, substitute=substitute)


class YAMLRetrieveSubstitution(launch.Substitution):
    _obj: YAMLFileSubstitution
    _key: list[launch.Substitution]

    def __init__(
        self,
        obj: YAMLFileSubstitution,
        key: launch.SomeSubstitutionsType,
    ):
        launch.Substitution.__init__(self)
        self._obj = obj
        self._key = launch.utilities.normalize_to_list_of_substitutions(key)

    def perform(self, context: launch.LaunchContext):
        obj = self._obj.perform_load(context)
        key = launch.utilities.perform_substitutions(context, self._key)

        remainder: str = key

        while len(remainder):
            if remainder in _yaml_iter(obj):
                return obj[remainder]

            newkey, *remainders = remainder.split(os.sep, 1)
            remainder = ''.join(remainders)

            try:
                newkey = int(newkey)
            except ValueError:
                pass

            obj = obj[newkey]

        return str(obj)


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


class _YAMLReplacer:
    @dataclasses.dataclass(frozen=True)
    class Replacement:
        value: typing.Any

    @dataclasses.dataclass(frozen=True)
    class NoReplacement(Replacement):
        value: typing.Any

    @dataclasses.dataclass(frozen=True)
    class StringReplacement(Replacement):
        value: typing.Any

    @dataclasses.dataclass(frozen=True)
    class DictSpreadReplacement(Replacement):
        value: dict

    @dataclasses.dataclass(frozen=True)
    class ListSpreadReplacement(Replacement):
        value: list

    _substitutions: dict

    def _replace_inter_string(self, v: str) -> NoReplacement | None:
        counter: int = 0
        replacements: list[tuple[int, int]] = []
        opening: int = 0
        for i, c in enumerate(v):
            if v[i:i + 2] == '${':
                if counter == 0:
                    opening = i
                counter += 1
            if c == '}':
                counter -= 1
                if counter < 0:
                    break
                if counter == 0:
                    replacements.append((opening, i + 1))

        if counter == 0 and replacements:

            # abandon inter-string substitution
            if replacements[0] == (0, len(v)):
                return None

            result = ''
            last_end: int = 0
            for start, end in replacements:
                result += v[last_end:start]

                matchable = v[start:end]

                match = self._sub_match(matchable)

                if not isinstance(match.value, str):
                    raise ValueError(f'misplaced substitution {matchable} of type {type(match.value)} in {v}')

                result += match.value
                last_end = end

            result += v[last_end:]
            return self.NoReplacement(value=result)

        return None

    def _sub_match(self, v: str) -> "Replacement":

        str_v: str | None = v
        replacement: None | _YAMLReplacer.Replacement = None

        while str_v is not None:
            if (match := re.match(r'^\$\{(.*)\}$', str_v)) is None:
                return self.NoReplacement(value=str_v)

            sub, *defaults = match.group(1).split(':-', 1)
            default = defaults[0] if defaults else None

            if sub.startswith('**'):
                if isinstance((substitution := self._substitutions.get(sub[len('**'):])), dict):
                    return self.DictSpreadReplacement(value=substitution)

            if sub.startswith('*'):
                if isinstance((substitution := self._substitutions.get(sub[len('*'):])), list):
                    return self.ListSpreadReplacement(value=substitution)

            if sub in self._substitutions:
                return self.StringReplacement(value=self._substitutions[sub])

            str_v = default

        if replacement is None:
            raise ValueError(f'could not find substitution for {v}')

        return replacement

    def _replace_list(self, obj: list) -> list:
        to_insert: list[tuple[int, list]] = []

        for k, v in enumerate(obj):
            if isinstance(v, str):
                replacement = self._sub_match(v)

                if isinstance(replacement, self.DictSpreadReplacement):
                    raise ValueError('dict spread argument placed outside dict')
                elif isinstance(replacement, self.ListSpreadReplacement):
                    to_insert.append((k, replacement.value))
                    continue
                elif isinstance(replacement, self.StringReplacement):
                    obj[k] = replacement.value

            obj[k] = self.replace(obj[k])

        offset: int = 0
        for i, insertions in to_insert:
            expanded = self._replace_list(insertions)
            obj.pop(i + offset)
            obj[i + offset:i + offset] = expanded
            offset += len(expanded) - 1

        return obj

    def _replace_dict(self, obj: dict) -> dict:
        to_insert: list[tuple[str, dict]] = []

        for k, v in obj.items():

            if isinstance(replacement := self._sub_match(k), self.DictSpreadReplacement):
                to_insert.append((k, replacement.value))
                continue

            if isinstance(v, str):
                replacement = self._sub_match(v)

                if isinstance(replacement, self.DictSpreadReplacement):
                    raise ValueError('dict spreads should be placed in dict keys')
                elif isinstance(replacement, self.ListSpreadReplacement):
                    raise ValueError('list spread argument placed outside list')
                elif isinstance(replacement, self.StringReplacement):
                    obj[k] = replacement.value

            obj[k] = self.replace(v)

        for key, insertions in to_insert:
            obj.pop(key)
            obj.update(self._replace_dict(insertions))

        return obj

    def _replace_str(self, obj: str) -> typing.Any:
        replacement = self._sub_match(obj)
        if (inter_v := self._replace_inter_string(obj)) is not None:
            return self.replace(inter_v.value)
        if isinstance(replacement, self.DictSpreadReplacement):
            raise ValueError('dict spread argument placed outside dict')
        elif isinstance(replacement, self.ListSpreadReplacement):
            raise ValueError('list spread argument placed outside list')
        elif isinstance(replacement, self.StringReplacement):
            return self.replace(replacement.value)
        elif isinstance(replacement, self.NoReplacement):
            return replacement.value
        return obj

    @typing.overload
    def replace(self, obj: dict) -> dict: ...

    @typing.overload
    def replace(self, obj: list) -> list: ...

    @typing.overload
    def replace(self, obj: str) -> str: ...

    def replace(self, obj: typing.Any) -> typing.Any:
        if isinstance(obj, list):
            return self._replace_list(obj)
        if isinstance(obj, dict):
            return self._replace_dict(obj)
        if isinstance(obj, str):
            return self._replace_str(obj)
        return obj

    def __init__(self, substitutions: dict):
        self._substitutions = substitutions


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

        replacer = _YAMLReplacer(substitutions)

        result = replacer.replace(self._obj.perform_load(context))

        return YAMLFileSubstitution.from_dict(
            result,
            substitute=True,
        ).perform(context)


class CurrentNamespaceSubstitution(launch.Substitution):
    def perform(self, context: launch.LaunchContext) -> typing.Text:
        """Perform the substitution."""
        return context.launch_configurations.get('ros_namespace', '/')
