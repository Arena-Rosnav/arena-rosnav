from typing import Type
from ..collectors import BaseUnit

__all__ = ["DoneObservation"]


class DoneObservation(BaseUnit):
    name: str = "done"
    data_class: Type[bool] = bool
