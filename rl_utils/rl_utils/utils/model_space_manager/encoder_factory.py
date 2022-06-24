from .base_space_encoder import BaseSpaceEncoder

class BaseSpaceEncoderFactory:
    """The factory class for creating space encoders"""

    registry = {}
    """ Internal registry for available space encoders """

    @classmethod
    def register(cls, name):
        """Class method to register agent class to the internal registry.

        Args:
            name (str): The name of the encoder.

        Returns:
            The agent class itself.
        """

        def inner_wrapper(wrapped_class):
            assert name not in cls.registry, f"Space encoder '{name}' already exists!"
            assert issubclass(wrapped_class, BaseSpaceEncoder)

            cls.registry[name] = wrapped_class
            return wrapped_class

        return inner_wrapper

    @classmethod
    def instantiate(cls, name: str, *kwargs):
        """Factory command to create the agent.
        This method gets the appropriate agent class from the registry
        and creates an instance of it, while passing in the parameters
        given in ``kwargs``.

        Args:
            name (str): The name of the agent to create.

        Returns:
            An instance of the agent that is created.
        """
        assert name in cls.registry, f"BaseSpaceEncoder '{name}' is not registered!"
        agent_class = cls.registry[name]
        
        if issubclass(agent_class, BaseSpaceEncoder):
            return agent_class(*kwargs)
        else:
            return agent_class
