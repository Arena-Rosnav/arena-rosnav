import heapq
import itertools
from typing import Callable, Iterator

from task_generator.shared import Model, ModelType, ModelWrapper  # noqa

from .models import ModelLoader  # noqa


class NamespaceIndexer:

    _freed: list[int]
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

    def __next__(self) -> tuple[str, Callable[[], None]]:
        index = self.get()
        return self.format(index), lambda: self.free(index)
