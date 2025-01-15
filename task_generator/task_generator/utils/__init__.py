from .models import ModelLoader
import functools
import subprocess
from typing import Callable, Collection, Dict, Iterator, List, Optional, Set, Tuple, Type

import os

import heapq
import itertools

from task_generator.shared import ModelWrapper, Model, ModelType


class NamespaceIndexer:

    _freed: List[int]
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

    def __next__(self) -> Tuple[str, Callable[[], None]]:
        index = self.get()
        return self.format(index), lambda: self.free(index)
