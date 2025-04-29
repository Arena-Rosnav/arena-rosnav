import typing

Key = typing.TypeVar('Key')
Value = typing.TypeVar('Value')


class Registry(typing.Generic[Key, Value]):

    __registry: dict[Key, typing.Callable[[], type[Value]]]
    __name: str

    def __init__(self, entries: typing.Optional[dict[Key, type[Value]]] = None) -> None:
        if entries is None:
            entries = {}

        self.__name = Value.__name__
        self.__registry = {**entries}

    def register(self, name: Key):
        def inner_wrapper(class_loader: typing.Callable[[], type[Value]]):
            assert name not in self.__registry, f"{self.__name} '{name}' already exists!"

            self.__registry[name] = class_loader
            return class_loader

        return inner_wrapper

    def get(self, name: Key) -> type[Value]:
        assert name in self.__registry, f"{self.__name} '{name}' is not registered!"

        instance = self.__registry[name]()

        return instance
