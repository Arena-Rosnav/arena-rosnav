import os
import typing

import rclpy


def DefaultParameter(value: typing.Any) -> rclpy.Parameter | None:
    if value is None:
        return None
    # if isinstance(value, rclpy.Parameter.Type):
    #     return None
    return rclpy.Parameter(
        '',
        value=value,
    )


class Namespace(str):
    def __call__(self, *args: str) -> "Namespace":
        return Namespace(os.path.join(self, *args)).remove_double_slash()

    def ParamNamespace(self) -> "ParamNamespace":
        return ParamNamespace('')(*self.split('/'))

    @property
    def simulation_ns(self) -> "Namespace":
        if len(self.split("/")) < 3:
            return self
        return Namespace(os.path.dirname(self))

    @property
    def robot_ns(self) -> "Namespace":
        return Namespace(os.path.basename(os.path.normpath(self)))

    def remove_double_slash(self) -> "Namespace":
        return Namespace(self.replace("//", "/"))


class ParamNamespace(Namespace):
    def __call__(self, *args: str) -> "ParamNamespace":
        return ParamNamespace(
            '.'.join((
                *((self,) if self else []),
                *args)
            )
        )

    def SlashNamespace(self) -> "Namespace":
        return Namespace('')(*self.split('.'))


# add representer if yaml is installed
try:
    import yaml
    yaml.add_representer(
        Namespace,
        lambda dumper,
        data: dumper.represent_str(
            str(data)))
except ImportError:
    pass
