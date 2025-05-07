import math
import os

import arena_simulation_setup
import rclpy
# Import dependencies.
from isaacsim_msgs.msg import Values
from isaacsim_msgs.srv import (DeletePrim, GetPrimAttributes, ImportObstacles,
                               ImportUsd, MovePrim, Pedestrian, SpawnWall,
                               UrdfToUsd)

from task_generator.shared import ModelType
from task_generator.simulators import BaseSimulator

# from omni.isaac.core.utils.rotations import euler_angles_to_quat


class IsaacSimulator(BaseSimulator):

    def init_service_clients(self):
        """
        Initialize all ROS 2 service clients and wait for their availability.
        """
        self._logger.info("Initializing service clients...")
        self.client = {}
        # Define services with their corresponding client attributes
        services = {
            'urdf_to_usd': {
                'service_type': UrdfToUsd,
                'service_name': 'isaac/urdf_to_usd',
                'client_attr': 'urdf_to_usd_client'
            },
            'import_usd': {
                'service_type': ImportUsd,
                'service_name': 'isaac/import_usd',
                'client_attr': 'spawn_entity_client'
            },
            'delete_prim': {
                'service_type': DeletePrim,
                'service_name': 'isaac/delete_prim',
                'client_attr': 'delete_entity_client'
            },
            'get_prim_attributes': {
                'service_type': GetPrimAttributes,
                'service_name': 'isaac/get_prim_attributes',
                'client_attr': 'get_entity_attributes_client'
            },
            'move_prim': {
                'service_type': MovePrim,
                'service_name': 'isaac/move_prim',
                'client_attr': 'move_entity_client'
            },
            'spawn_wall': {
                'service_type': SpawnWall,
                'service_name': 'isaac/spawn_wall',
                'client_attr': 'spawn_wall_client'
            },
            'import_obstacle': {
                'service_type': ImportObstacles,
                'service_name': 'isaac/import_obstacle',
                'client_attr': 'spawn_obstacle_client'
            },
            'import_pedestrians': {
                'service_type': Pedestrian,
                'service_name': 'isaac/spawn_pedestrian',
                'client_attr': 'spawn_pedestrian_client'
            },
        }

        # Initialize and wait for each service client
        for service_key, service_info in services.items():
            service_type = service_info['service_type']
            service_name = service_info['service_name']
            client_attr = service_info['client_attr']

            # Create the service client
            self.client[client_attr] = self.node.create_client(service_type, service_name)

            self._logger.info(f'Waiting for service "{service_name}"...')

            # Wait for the service to become available
            while not self.client[client_attr].wait_for_service(timeout_sec=10.0):
                self._logger.error(f'Service "{service_name}" not available after waiting')
                # raise TimeoutError(f'Service "{service_name}" not available')

            self._logger.info(f'Service "{service_name}" is now available.')

        self._logger.info("All service clients initialized and available.")

    def spawn_entity(self, entity):
        self._logger.info(
            f"Attempting to spawn model: {entity.name}"
        )

        # self._logger.info(entity.position)
        model = entity.model.get(
            [ModelType.URDF, ModelType.USD],
            loader_args=entity.asdict(),
        )
        if model.type == ModelType.URDF:
            robot_params = arena_simulation_setup.Robot(entity.model.name)
            response = self.client['urdf_to_usd_client'].call(
                UrdfToUsd.Request(
                    name=entity.name,
                    urdf_path=os.path.abspath(model.path),
                    no_localization=False,
                    base_frame=robot_params.base_frame,
                    odom_frame=robot_params.odom_frame,
                )
            )
            # response = self.client['move_entity_client'].call_async(
            # MovePrim.Request(
            #     name=entity.name,
            #     prim_path=f"/{entity.name}",
            #     values=[
            #         Values(values=[entity.position.x,entity.position.y,0.1]),
            #         Values(values=[0.0,0.0,0.0])]
            #     )
            # )
            return bool(response.usd_path)
        else:
            usd_path = os.path.abspath(model.path)
            # self._logger.info(usd_path)
            response = self.client['spawn_obstacle_client'].call_async(
                ImportObstacles.Request(
                    name=entity.name,
                    usd_path=usd_path,
                    position=[entity.position.x, entity.position.y, 0.12],
                    orientation=[0.0, 0.0, entity.position.orientation],
                )
            )
            return True
            # TODO
            rclpy.spin_until_future_complete(self.node, response)
            return response.result().ret

    def move_entity(self, name, position):
        self._logger.info(
            f"Attempting to move entitiy: {name}"
        )

        self._logger.info(f"position: {position.x,position.y}")
        self._logger.info(f"orientation: {position.orientation}")
        prim_path = f"/{name}"

        response = self.client['move_entity_client'].call_async(
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

        response = self.client['delete_entity_client'].call_async(
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
                future = self.client['spawn_wall_client'].call_async(
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
