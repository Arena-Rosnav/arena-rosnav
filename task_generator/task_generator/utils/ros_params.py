import traceback
import typing
import abc

import rclpy
import rclpy.exceptions
import rclpy.node
import rcl_interfaces.msg

from task_generator.shared import DefaultParameter

T = typing.TypeVar('T')
U = typing.TypeVar('U')


class ROSParamT(abc.ABC, typing.Generic[T]):

    @abc.abstractmethod
    def __init__(
        self,
        /,
        name: str,
        value: typing.Any,
        *,
        type_: typing.Optional[rclpy.Parameter.Type] = None,
        parse: typing.Optional[typing.Callable[[
            typing.Any], T]] = None,
        **kwargs,
    ) -> None:
        ...

    @property
    @abc.abstractmethod
    def name(self) -> str:
        """
        Get name.
        """

    @property
    @abc.abstractmethod
    def value(self) -> T:
        """
        Get cached value.
        """

    @value.setter
    @abc.abstractmethod
    def value(self, value):
        """
        Set value and publish.
        """

    @abc.abstractmethod
    def callback(self, value: typing.Any) -> bool:
        """
        Callback function for setting value.
        """


class _ROSParam(ROSParamT[T], typing.Generic[T]):
    """
    Wrapper that handles callbacks.
    """

    _node: typing.ClassVar["ROSParamServer"]
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

    @property
    def name(self) -> str:
        return self._name

    @property
    def value(self) -> T:
        return self._value

    @value.setter
    def value(self, value: typing.Any):
        self._node.set_parameters([
            rclpy.Parameter(
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

    def __init__(
        self,
        /,
        name: str,
        value: typing.Optional[typing.Any] = None,
        *,
        type_: typing.Optional[rclpy.Parameter.Type] = None,
        parse: typing.Optional[typing.Callable[[typing.Any], T]] = None,
        **kwargs,
    ) -> None:
        self._name = name

        if parse is None:
            parse = self.identity
        self._from_param = parse

        if type_ is not None:
            self._node.rosparam.declare_safe(
                self.name,
                type_,
            )

        self._node.register_param(self, value, **kwargs)


counter = 0


class _rosparam(typing.Generic[T]):
    """
    Light-weight stateless interface for singular typed rosparam actions (short-lived).
    Runtime checks are not performed.
    Use ROSParam instead for most use cases.
    """

    _node: typing.ClassVar["ROSParamServer"]

    _UNSET = typing.NewType('_UNSET', None)

    @classmethod
    def declare_safe(
        cls, param_name: str,
        value: typing.Any = None,
        **kwargs
    ) -> None:
        if cls._node.has_parameter(param_name):
            return

        try:
            cls._node.declare_parameter(param_name, value, **kwargs)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

    @classmethod
    def get_unsafe(
        cls,
        param_name: str,
        default: T | typing.Type[_UNSET] = _UNSET
    ) -> T:
        """
        Get value of parameter.
        """

        _default = default

        default_value = None
        if default is not cls._UNSET:
            default_value = DefaultParameter(default)

        result = cls._node.get_parameter_or(
            param_name,
            default_value,
        )
        if result.type_ is rclpy.Parameter.Type.NOT_SET:
            if _default is not cls._UNSET:
                return _default
            raise ValueError(
                f'parameter {param_name} is unset and no default passed'
            )

        return result.value

    @classmethod
    def get(cls, param_name: str, default: T) -> T:
        """
        Get value of parameter. Declare if undeclared.
        """
        if default is not None:
            cls.declare_safe(param_name, default)
        return cls.get_unsafe(param_name, default)

    @classmethod
    def set_unsafe(cls, param_name: str, value: T) -> bool:
        """
        Set value of parameter.
        """

        return cls._node.set_parameters([
            rclpy.Parameter(param_name, value=value)
        ])[0].successful

    @classmethod
    def set(cls, param_name: str, value: T) -> bool:
        """
        Set value of parameter. Declare if undeclared.
        """

        try:
            return cls.set_unsafe(param_name, value)
        except rclpy.exceptions.ParameterNotDeclaredException:
            cls._node.declare_parameter(param_name, value)
            return True

    @classmethod
    def callback(
        cls,
        param_name: str,
        callback: typing.Callable[[typing.Any], bool]
    ):
        try:
            value = cls.get_unsafe(param_name)
        except ValueError:
            value = None

        cls._node._callbacks.setdefault(param_name, set()).add(callback)

        if value is not None:
            if cls._node.executor is not None:
                cls._node.executor.create_task(lambda: callback(value))
            else:
                callback(value)


class ROSParamServer(rclpy.node.Node):

    # this confuses my type checker
    # ROSParam: typing.Type[_ROSParam[typing.Any]]
    # rosparam: typing.Type[_rosparam[typing.Any]]

    _callbacks: typing.Dict[
        str,
        typing.Set[typing.Callable[[typing.Any], bool]]
    ]

    def register_param(self, param: ROSParamT[T], value: typing.Any, **kwargs):

        current_value = self.rosparam[T].get(
            param.name,
            value
        )

        self._callbacks.setdefault(param.name, set()).add(param.callback)

        result = self._callback([
            rclpy.Parameter(
                name=param.name,
                value=current_value
            )
        ])

        if not result.successful:
            raise RuntimeError(
                f'initial configuration of parameter {param.name} failed with {result.reason}')

    def _callback(self, params: list[rclpy.Parameter]):
        successful = True
        reason: list[str] = []
        for param in params:
            for callback in self._callbacks.get(param.name, set()):
                self.get_logger().debug(
                    f"setting param {param.name} with value {param.value} (callback {callback})")
                try:
                    successful &= callback(param.value)
                except BaseException as e:
                    self.get_logger().warn(
                        f'setting parameter {param.name} with value {param.value} failed: {e}')
                    reason.append(
                        ''.join(
                            traceback.TracebackException.from_exception(e).format()))
                    successful = False

        if not successful:
            # this function can cause inconsistent states when some callbacks
            # succeed, some fail. revert back to old value here.
            ...

        return rcl_interfaces.msg.SetParametersResult(
            successful=successful,
            reason="\n".join(reason)
        )

    def __init__(self):
        self._callbacks = {}
        self.add_on_set_parameters_callback(self._callback)
        self._setup_rosparam()

    def _setup_rosparam(self):
        class ROSParam_impl(_ROSParam[T], typing.Generic[T]):
            _node = self
        self.ROSParam = ROSParam_impl

        class rosparam_impl(_rosparam[T], typing.Generic[T]):
            _node = self
        self.rosparam = rosparam_impl
