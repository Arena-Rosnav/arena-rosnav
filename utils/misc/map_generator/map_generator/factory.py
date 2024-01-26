from .base_map_gen import BaseMapGenerator


class MapGeneratorFactory:
    """
    Factory class for creating instances of map generators.
    """

    registry = {}

    @classmethod
    def register(cls, name):
        """
        Decorator function to register a map generator class with a given name.

        Args:
            name (str): The name of the map generator.

        Returns:
            The decorated map generator class.
        """

        def inner_wrapper(wrapped_class):
            assert name not in cls.registry, f"Generator '{name}' already exists!"
            assert issubclass(wrapped_class, BaseMapGenerator)

            cls.registry[name] = wrapped_class
            return wrapped_class

        return inner_wrapper

    @classmethod
    def instantiate(cls, name: str, *args, **kwargs):
        """
        Instantiate a map generator with the given name.

        Args:
            name (str): The name of the map generator.
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.

        Returns:
            An instance of the map generator.

        Raises:
            AssertionError: If the specified generator name is not registered.
        """
        assert name in cls.registry, f"Generator '{name}' is not registered!"
        generator_class = cls.registry[name]

        return generator_class(*args, **kwargs)
