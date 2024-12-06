import functools
import typing
import attr

import builtin_interfaces.msg


@functools.total_ordering
@attr.define
class Time:
    """
    Wrapper for builtin_interfaces.msg.Time
    """
    sec: int = attr.field(converter=int, default=0)
    nanosec: int = attr.field(converter=int, default=0)

    def __eq__(self, value: object) -> bool:
        if not isinstance(value, (type(self), builtin_interfaces.msg.Time)):
            return False
        return self.sec == value.sec and self.nanosec == value.nanosec

    def __lt__(self, value: object) -> bool:
        if not isinstance(value, (type(self), builtin_interfaces.msg.Time)):
            return False
        return self.sec < value.sec or self.nanosec < value.nanosec

    @classmethod
    def from_time(cls, v: builtin_interfaces.msg.Time) -> typing.Self:
        """
        Create instance from builtin_interfaces.msg.Time object.
        """
        return cls(
            sec=v.sec,
            nanosec=v.nanosec,
        )

    def to_time(self) -> builtin_interfaces.msg.Time:
        """
        Create builtin_interfaces.msg.Time from self.
        """
        return builtin_interfaces.msg.Time(
            sec=self.sec,
            nanosec=self.nanosec,
        )
