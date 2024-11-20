import functools
import typing
import abc
import rclpy
import rcl_interfaces.msg


class RosParam(abc.ABC):
    @property
    @abc.abstractmethod
    def value(self) -> typing.Any:
        """
        Get cached value.
        """

    @value.setter
    @abc.abstractmethod
    def set_value(self, value):
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
        ) -> None:
            self._name = name

            if parse is None:
                parse = self.identity
            self._from_param = parse

            self._parameter_value = value
            print(parse)
            self._value = parse(self._parameter_value)
            self._node.register_param(self)

        @property
        def name(self) -> str:
            return self._name

        @property
        def value(self) -> T:
            return self._value

        @value.setter
        def set_value(self, value: typing.Any):
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
        def set_param(self, value: typing.Any) -> bool:
            self._value = self._from_param(value)
            return True

    __callbacks: typing.Dict[
        str,
        typing.Set[typing.Callable[[typing.Any], bool]]
    ]

    @property
    def ROSParam(self) -> typing.Type["_ROSParam"]:
        class ROSParam[T](self._ROSParam[T]):
            _node = self
        return ROSParam

    def register_param(self, param: _ROSParam):
        if param.name not in self.__callbacks:
            self.__callbacks[param.name] = set()
            self.declare_parameter(param.name, param.param)
        self.__callbacks[param.name].add(param.set_param)

    def _callback(self, params):
        successful = True
        for param in params:
            for callback in self.__callbacks.get(param.name, set()):
                successful &= callback(param.value)
        return rcl_interfaces.msg.SetParametersResult(successful=successful)

    def __init__(self):
        self.__callbacks = {}
        self.add_on_set_parameters_callback(self._callback)
