
import enum

import attrs
import yaml


class MappingDirection(str, enum.Enum):
    BIDIRECTIONAL = '@'
    GZ_TO_ROS = '['
    ROS_TO_GZ = ']'


@attrs.define
class _TopicMapping(dict[str, str]):
    gz_topic: str
    ros_topic: str
    ros_type: str
    gz_type: str
    direction: MappingDirection = attrs.field(converter=MappingDirection)

    def substitute(self, subs: dict[str, str]) -> "_TopicMapping":
        return _TopicMapping(**{
            k: v.format(**subs)
            for k, v
            in self.as_dict().items()
        })

    def as_arg(self) -> str:
        return f"{self.gz_topic}@{self.ros_type}{self.direction.value}{self.gz_type}"

    def as_remapping(self) -> tuple[str, str]:
        return (self.gz_topic, self.ros_topic)

    def as_dict(self) -> dict[str, str]:
        return attrs.asdict(
            self,
            value_serializer=lambda _, __, v: v.value if isinstance(v, MappingDirection) else v
        )

    def as_yaml_dict(self) -> dict[str, str]:
        return attrs.asdict(
            self,
            value_serializer=lambda _, __, v: v.name if isinstance(v, MappingDirection) else v
        )


class BridgeConfiguration(list[_TopicMapping]):
    @classmethod
    def from_file(cls, path: str) -> "BridgeConfiguration":
        with open(path, 'r') as f:
            return BridgeConfiguration([
                _TopicMapping(**mapping)
                for mapping
                in yaml.safe_load(f)
            ])

    def substitute(self, subs: dict[str, str]) -> "BridgeConfiguration":
        return BridgeConfiguration([mapping.substitute(subs) for mapping in self])

    def as_args(self) -> list[str]:
        return list(map(_TopicMapping.as_arg, self))

    def as_remappings(self) -> list[tuple[str, str]]:
        return list(map(_TopicMapping.as_remapping, self))

    def as_yaml(self) -> str:
        return yaml.dump(list(map(_TopicMapping.as_yaml_dict, self)))
