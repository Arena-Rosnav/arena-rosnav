from __future__ import annotations
from typing import List, Optional, Union


class Namespace:
    """
    Represents a namespace for topics in ROS (Robot Operating System).

    A namespace is a way to organize topics in ROS. It is represented as a list of strings,
    where each string represents a level in the namespace hierarchy.

    Attributes:
        name (List[str]): The list of strings representing the namespace hierarchy.

    Methods:
        __init__(self, name: Union[str, Namespace]): Initializes a new Namespace object.
        __str__(self) -> str: Returns the string representation of the namespace.
        __repr__(self) -> str: Returns the string representation of the namespace for debugging purposes.
        __truediv__(self, other: Union[str, Namespace, Topic]) -> Topic: Implements the '/' operator for creating topics within the namespace.
        __call__(self, *args: str) -> Topic: Implements the function call operator for creating topics within the namespace.
        __contains__(self, item: Union[str, Namespace]) -> bool: Implements the 'in' operator for checking if a topic or namespace is within the namespace.
        simulation_ns(self) -> Namespace: Returns the simulation namespace.
        robot_ns(self) -> Namespace: Returns the robot namespace.
    """

    def __init__(self, name: Union[str, Namespace]):
        self.name: List[str] = (
            name.name
            if isinstance(name, Namespace)
            else [ns for ns in name.split("/") if ns]
        )

    def __str__(self) -> str:
        return f"/{'/'.join(self.name)}"

    def __repr__(self) -> str:
        return f"Namespace['{self.name}']"

    def __truediv__(self, other: Union[str, Namespace, Topic]) -> Topic:
        if isinstance(other, str):
            return Topic(other, namespaces=[self])
        elif isinstance(other, Namespace):
            return Topic("", namespaces=[self] + ([other] if other.name else []))
        elif isinstance(other, Topic):
            return other.prepend_namespace(self)
        else:
            raise TypeError(f"Unsupported operand type for /: '{type(other)}'")

    def __call__(self, *args: str) -> Topic:
        return Topic("/".join(self.name + list(map(lambda x: str(x).strip("/"), args))))

    def __contains__(self, item: Union[str, Namespace]) -> bool:
        if isinstance(item, str):
            return item in self.name
        elif isinstance(item, Namespace):
            return all(ns in self.name for ns in item.name)
        else:
            raise TypeError(f"Unsupported operand type for 'in': '{type(item)}'")

    @property
    def simulation_ns(self) -> Namespace:
        return Namespace(self.name[0] if self.name else "")

    @property
    def robot_ns(self) -> Namespace:
        return Namespace(self.name[1] if len(self.name) > 1 else "")


class Topic:
    """
    Represents a ROS topic.

    Args:
        name (str): The name of the topic.
        namespaces (Optional[Union[str, List[Union[str, Namespace]], Namespace]]): The namespaces associated with the topic.

    Attributes:
        namespaces (List[Namespace]): The list of namespaces associated with the topic.
        name (str): The name of the topic.
        additional_namespaces (List[Namespace]): Additional namespaces associated with the topic.

    """

    def __init__(
        self,
        name: str,
        namespaces: Optional[Union[str, List[Union[str, Namespace]], Namespace]] = None,
    ):
        self.namespaces: List[Namespace] = self._process_namespaces(namespaces)
        self.name, additional_namespaces = self._process_name(name)
        self.namespaces.extend(additional_namespaces)

    def _process_namespaces(
        self, namespaces: Optional[Union[str, List[Union[str, Namespace]], Namespace]]
    ) -> List[Namespace]:
        """
        Process the namespaces associated with the topic.

        Args:
            namespaces (Optional[Union[str, List[Union[str, Namespace]], Namespace]]): The namespaces to process.

        Returns:
            List[Namespace]: The processed list of namespaces.

        Raises:
            TypeError: If the type of 'namespaces' is not supported.

        """
        if isinstance(namespaces, str):
            return [Namespace(ns) for ns in namespaces.split("/") if ns]
        elif isinstance(namespaces, list):
            return [Namespace(ns) if isinstance(ns, str) else ns for ns in namespaces]
        elif isinstance(namespaces, Namespace):
            return [namespaces]
        elif namespaces is None:
            return []
        else:
            raise TypeError(f"Unsupported type for 'namespaces': '{type(namespaces)}'")

    def _process_name(self, name: str) -> tuple[str, List[Namespace]]:
        """
        Process the name of the topic.

        Args:
            name (str): The name of the topic.

        Returns:
            tuple[str, List[Namespace]]: The processed name and additional namespaces.

        """
        parts = name.strip("/").split("/")
        if len(parts) < 2:
            return parts[0], []
        return parts[-1], [Namespace(ns) for ns in parts[:-1] if ns]

    def __str__(self) -> str:
        return self.full_topic

    def __repr__(self) -> str:
        return f"Topic['{self.name}', namespace='{''.join(str(ns) for ns in self.namespaces)}']"

    def __truediv__(self, other: Union[str, Namespace, Topic]) -> Topic:
        if isinstance(other, str):
            new_name = f"{self.name}/{other}" if self.name else other
            return Topic(new_name, namespaces=self.namespaces)
        elif isinstance(other, Namespace):
            return Topic(
                self.name, namespaces=self.namespaces + ([other] if other.name else [])
            )
        elif isinstance(other, Topic):
            if self.name:
                raise ValueError("Cannot append a topic to a non-root topic")
            return Topic(other.name, namespaces=self.namespaces + other.namespaces)
        else:
            raise TypeError(f"Unsupported operand type for /: '{type(other)}'")

    def __contains__(self, item: Union[str, Namespace]) -> bool:
        if isinstance(item, str):
            return item in self.full_topic
        elif isinstance(item, Namespace):
            return all(ns in self.full_topic for ns in item.name)
        else:
            raise TypeError(f"Unsupported operand type for 'in': '{type(item)}'")

    def prepend_namespace(self, namespace: Union[str, Namespace]) -> Topic:
        if not isinstance(namespace, Namespace) or not isinstance(namespace, str):
            raise TypeError(
                f"Unsupported operand type for prepend_namespace: '{type(namespace)}'"
            )

        if isinstance(namespace, str):
            namespace = Namespace(namespace)

        return Topic(self.name, namespaces=[namespace] + self.namespaces)

    @classmethod
    def from_full_topic(cls, full_topic: str) -> Topic:
        parts = full_topic.strip("/").split("/")
        return cls(parts[-1], namespaces=parts[:-1])

    @property
    def full_topic(self) -> str:
        return "".join(str(ns) for ns in self.namespaces) + "/" + self.name

    @property
    def simulation_ns(self) -> str:
        return str(self.namespaces[0]) if self.namespaces else ""

    @property
    def robot_ns(self) -> str:
        return str(self.namespaces[1]) if len(self.namespaces) > 1 else ""


StringNamespace = str
ListOfNamespaces = List[Union[str, Namespace]]
