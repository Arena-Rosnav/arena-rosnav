import typing
import abc
import rclpy
import rcl_interfaces.msg
from task_generator.shared import DefaultParameter


class RosParam(abc.ABC):
    @property
    @abc.abstractmethod
    def value(self) -> typing.Any:
        """
        Get cached value.
        """

    @value.setter
    @abc.abstractmethod
    def value(self, value):
        """
        Set value and publish.
        """


U = typing.TypeVar('U')


class ROSParamServer(rclpy.node.Node):

    class _ROSParam[T](RosParam, abc.ABC):
        """
        Wrapper that handles callbacks.
        """

        _node: "ROSParamServer"
        _name: str

        _type: typing.TypeVar
        _from_param: typing.Callable[[typing.Any], T]

        _value: T
        _parameter_value: typing.Any

        @staticmethod
        def identity(x, *args):
            """
            lambda x: x
            """
            return x

        def __init__(
            self,
            /,
            name: str,
            value: typing.Any,
            *,
            parse: typing.Optional[typing.Callable[[
                typing.Any], T]] = None,
            **kwargs,
        ) -> None:
            self._name = name

            if parse is None:
                parse = self.identity
            self._from_param = parse

            self._node.register_param(self, value, **kwargs)
            # self._parameter_value = value
            # self._value = parse(self._parameter_value)

        @property
        def name(self) -> str:
            return self._name

        @property
        def value(self) -> T:
            return self._value

        @value.setter
        def value(self, value: typing.Any):
            self._node.set_parameters([
                rclpy.parameter.Parameter(
                    name=self._name,
                    value=value
                )
            ])

        @property
        def param(self) -> typing.Any:
            return self._parameter_value

        @param.setter
        def param(self, value: typing.Any):
            self._value = self._from_param(value)

        def callback(self, value: typing.Any) -> bool:
            self.param = value
            return True

    __callbacks: typing.Dict[
        str,
        typing.Set[typing.Callable[[typing.Any], bool]]
    ]

    def register_param(self, param: _ROSParam, value: typing.Any, **kwargs):

        parameter_known = param.name in self.__callbacks

        self.__callbacks.setdefault(param.name, set()).add(param.callback)

        if not parameter_known:
            try:
                self.declare_parameter(param.name, value, **kwargs)
            except rclpy.exceptions.ParameterAlreadyDeclaredException:
                pass

    def _callback(self, params):
        successful = True
        for param in params:
            for callback in self.__callbacks.get(param.name, set()):
                successful &= callback(param.value)
        return rcl_interfaces.msg.SetParametersResult(successful=successful)

    @property
    def ROSParam(self) -> typing.Type["_ROSParam"]:
        """
        Typed ROS2 parameter class with callbacks.
        """
        class ROSParam[T](self._ROSParam[T]):
            _node = self
        return ROSParam

    class rosparam[T]:
        """
        Light-weight stateless interface for singular typed rosparam actions (short-lived).
        Runtime checks are not performed.
        Use ROSParam instead for most use cases.
        """

        _node: "ROSParamServer"

        class _unspecified(object):
            ...

        @classmethod
        def get(cls, param_name: str,
                default: "T | typing.Type[_unspecified]" = _unspecified) -> T:
            """
            Get value of parameter.
            """

            value = cls._node.get_parameter_or(
                param_name, DefaultParameter(default)
            ).value
            if value is cls._unspecified:
                raise ValueError(
                    f'parameter {param_name} is unset and no default passed'
                )

            return value

        @classmethod
        def set(cls, param_name: str, value: T) -> bool:
            """
            Set value of parameter.
            """

            return cls._node.set_parameters([
                rclpy.parameter.Parameter(param_name, value=value)
            ])[0].successful

    def __init__(self):
        self.rosparam._node = self
        self.__callbacks = {}
        self.add_on_set_parameters_callback(self._callback)
