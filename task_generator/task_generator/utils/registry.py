import typing

Key = typing.TypeVar('Key')
Value = typing.TypeVar('Value')


class Registry(typing.Generic[Key, Value]):

    __registry: typing.Dict[Key, typing.Callable[[], typing.Type[Value]]]
    __name: str

    def __init__(self, entries: typing.Optional[typing.Dict[Key, typing.Type[Value]]] = None) -> None:
        if entries is None:
            entries = {}

        self.__name = Value.__name__
        self.__registry = {**entries}

    def register(self, name: Key):
        def inner_wrapper(class_loader: typing.Callable[[], typing.Type[Value]]):
            assert name not in self.__registry, f"{self.__name} '{name}' already exists!"

            self.__registry[name] = class_loader
            return class_loader

        return inner_wrapper

    def get(self, name: Key) -> typing.Type[Value]:
        assert name in self.__registry, f"{self.__name} '{name}' is not registered!"

        instance = self.__registry[name]()

        return instance
