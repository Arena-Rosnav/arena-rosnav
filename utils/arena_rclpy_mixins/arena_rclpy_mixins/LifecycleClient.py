import os

import lifecycle_msgs.msg
import lifecycle_msgs.srv
import rclpy.node


class LifecycleClient(rclpy.node.Node):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def get_lifecycle_state(self, node_name: str, *, timeout: float | None = None, **kwargs) -> lifecycle_msgs.msg.State:
        """
        Get state of lifecycle node
        """
        cli = self.create_client(
            lifecycle_msgs.srv.GetState,
            name := os.path.join(node_name, 'get_state'),
            **kwargs,
        )
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'timed out waiting for {name} after {timeout} secs')
        return cli.call(lifecycle_msgs.srv.GetState.Request()).current_state

    def get_available_lifecycle_states(self, node_name: str, *, timeout: float | None = None, **kwargs) -> list[lifecycle_msgs.msg.State]:
        """
        Get available lifecycle states
        """
        cli = self.create_client(
            lifecycle_msgs.srv.GetAvailableStates,
            name := os.path.join(node_name, 'get_available_states'),
            **kwargs,
        )

        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'timed out waiting for {name} after {timeout} secs')
        return cli.call(lifecycle_msgs.srv.GetAvailableStates.Request()).available_states

    def get_available_lifecycle_transitions(self, node_name: str, *, timeout: float | None = None, **kwargs) -> list[lifecycle_msgs.msg.Transition]:
        """
        Get available lifecycle transitions
        """
        cli = self.create_client(
            lifecycle_msgs.srv.GetAvailableTransitions,
            name := os.path.join(node_name, 'get_available_transitions'),
            **kwargs,
        )

        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'timed out waiting for {name} after {timeout} secs')
        return cli.call(lifecycle_msgs.srv.GetAvailableTransitions.Request()).available_transitions

    def set_lifecycle_state(self, node_name: str, state: lifecycle_msgs.msg.State, *, timeout: float | None = None, **kwargs) -> bool:
        """
        Set state of lifecycle node
        """
        cli = self.create_client(
            lifecycle_msgs.srv.SetState,
            name := os.path.join(node_name, 'set_state'),
            **kwargs,
        )

        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'timed out waiting for {name} after {timeout} secs')
        return cli.call(lifecycle_msgs.srv.SetState.Request(state=state)).success
