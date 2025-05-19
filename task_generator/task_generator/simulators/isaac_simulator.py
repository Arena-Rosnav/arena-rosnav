import math
import os
import typing

import arena_simulation_setup
import attrs
import rclpy
# Import dependencies.
from isaacsim_msgs.msg import Person, Values
from isaacsim_msgs.srv import (DeletePrim, GetPrimAttributes, ImportObstacles,
                               ImportUsd, MovePrim, Pedestrian, SpawnWall,
                               UrdfToUsd)
from task_generator.shared import DynamicObstacle, ModelType, Obstacle, Robot
from task_generator.simulators import BaseSimulator

# from omni.isaac.core.utils.rotations import euler_angles_to_quat


@attrs.define()
class _Service:
    type_: typing.Any
    name: str

    _client: rclpy.client.Client = attrs.field(init=False)

    @property
    def client(self) -> rclpy.client.Client:
        if self._client is None:
            raise RuntimeError(f"client for service {self.name} not initialized")
        return self._client

    @client.setter
    def client(self, value: rclpy.client.Client):
        self._client = value


class _Services(typing.NamedTuple):
    get_prim_attributes: _Service
    urdf_to_usd: _Service
    import_usd: _Service
    import_obstacle: _Service
    move_prim: _Service
    delete_prim: _Service
    spawn_wall: _Service
    import_pedestrians: _Service


class IsaacSimulator(BaseSimulator):

    def init_service_clients(self):
        """
        Initialize all ROS 2 service clients and wait for their availability.
        """
        self._logger.info("Initializing service clients...")

        # Define services with their corresponding client attributes
        self.services = _Services(
            urdf_to_usd=_Service(
                type_=UrdfToUsd,
                name='isaac/urdf_to_usd'
            ),
            import_usd=_Service(
                type_=ImportUsd,
                name='isaac/import_usd'
            ),
            delete_prim=_Service(
                type_=DeletePrim,
                name='isaac/delete_prim'
            ),
            get_prim_attributes=_Service(
                type_=GetPrimAttributes,
                name='isaac/get_prim_attributes'
            ),
            move_prim=_Service(
                type_=MovePrim,
                name='isaac/move_prim'
            ),
            spawn_wall=_Service(
                type_=SpawnWall,
                name='isaac/spawn_wall'
            ),
            import_obstacle=_Service(
                type_=ImportObstacles,
                name='isaac/import_obstacle'
            ),
            import_pedestrians=_Service(
                type_=Pedestrian,
                name='isaac/spawn_pedestrian'
            ),
        )

        # Initialize and wait for each service client
        for service in self.services:

            # Create the service client
            service.client = self.node.create_client(service.type_, service.name)

            self._logger.info(f'Waiting for service "{service.name}"...')

            # Wait for the service to become available
            timeout_sec = 10.0
            while not service.client.wait_for_service(timeout_sec=timeout_sec):
                self._logger.warning(f'Service "{service.name}" not available after waiting {timeout_sec}s')
                # raise TimeoutError(f'Service "{service_name}" not available')

            self._logger.info(f'Service "{service.name}" is now available.')

        self._logger.info("All service clients initialized and available.")

    def spawn_entity(self, entity):
        self._logger.info(
            f"Attempting to spawn model: {entity.name}"
        )

        if isinstance(entity, DynamicObstacle):
            return self._spawn_pedestrian(entity)

        if isinstance(entity, Robot):
            return self._spawn_robot(entity)

        assert isinstance(entity, Obstacle)
        return self._spawn_obstacle(entity)

    def move_entity(self, name, position):
        self._logger.info(
            f"Attempting to move entitiy: {name}"
        )

        self._logger.info(f"position: {position.x,position.y}")
        self._logger.info(f"orientation: {position.orientation}")
        prim_path = f"/{name}"

        response = self.services.move_prim.client.call_async(
            MovePrim.Request(
                name=name,
                prim_path=f"/{name}",
                values=[
                    Values(values=[position.x, position.y, 0.12]),
                    Values(values=[0.0, 0.0, math.degrees(position.orientation)])]
            )
        )
        if response is None:
            raise RuntimeError(f'failed to move entity: service timed out')

        return True

    def delete_entity(self, name):
        self._logger.info(
            f"Attempting to delete prim named {name}"
        )

        prim_path = f"/World/{name}"

        response = self.services.delete_prim.client.call_async(
            DeletePrim.Request(
                name=name,
                prim_path=prim_path
            )
        )

        return True

        # TODO
        if response is None:
            raise RuntimeError(
                f'failed to delete entity: service timed out')

    def spawn_walls(self, walls):
        # return True
        self._logger.info(
            f"Attempting to spawn walls"
        )
        # print(walls)
        world_path = "/World"
        for i, wall in enumerate(walls):
            try:
                # print(f"wall {i+1}: {wall}")
                start = [wall.Start.x, wall.Start.y]
                end = [wall.End.x, wall.End.y]
                future = self.services.spawn_wall.client.call_async(
                    SpawnWall.Request(
                        name=f"wall_{i+1}",
                        world_path=world_path,
                        start=start,
                        end=end,
                        height=wall.height
                    )
                )

                self._logger.info(f"Successfully spawned wall {i+1}")

            except Exception as e:
                self._logger.error(str(e))
                raise  # Re-raise exception after logging

        self._logger.info("All walls spawned successfully.")
        return True

    # TODO: update
    def before_reset_task(self):
        return True

    # TODO: update
    def after_reset_task(self):
        return True

    def _spawn_pedestrian(self, pedestrian: DynamicObstacle) -> bool:
        # TODO
        # implement externally managed pedestrians
        return True

    def _spawn_robot(self, robot: Robot) -> bool:
        model = robot.model.get(
            [
                ModelType.URDF,
                # ModelType.USD
            ],
            loader_args=robot.asdict(),
        )
        if model.type == ModelType.URDF:
            robot_params = arena_simulation_setup.Robot(robot.model.name)
            response = self.services.urdf_to_usd.client.call(
                UrdfToUsd.Request(
                    name=robot.name,
                    urdf_path=os.path.abspath(model.path),
                    robot_model=robot.model.name,
                    no_localization=False,
                    base_frame=robot_params.base_frame,
                    odom_frame=robot_params.odom_frame,
                )
            )
            return True

        # TODO
        raise NotImplementedError(f"robot model of type {model.type} can't be spawned by {self.__class__.__name__}")

    def _spawn_obstacle(self, obstacle: Obstacle) -> bool:
        model = obstacle.model.get([ModelType.USD])
        usd_path = os.path.abspath(model.path)
        response = self.services.import_obstacle.client.call_async(
            ImportObstacles.Request(
                name=obstacle.name,
                usd_path=usd_path,
                position=[obstacle.position.x, obstacle.position.y, 0.12],
                orientation=[0.0, 0.0, obstacle.position.orientation],
            )
        )
        return True

    def __init__(self, namespace):
        """Initialize IsaacSimulator

        Args:
            namespace: Namespace for the simulator
        """

        self._logger.info(
            f"Initializing IsaacSimulator with namespace: {namespace}")

        self.init_service_clients()

        self._logger.info(
            f"Done initializing Isaac Sim")
