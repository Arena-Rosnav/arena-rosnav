from task_generator.simulators import BaseSimulator


class IsaacSimulator(BaseSimulator):
    def __init__(self, namespace):
        """Initialize GazeboSimulator

        Args:
            namespace: Namespace for the simulator
        """

        super().__init__(namespace=namespace)
        raise NotImplementedError()
