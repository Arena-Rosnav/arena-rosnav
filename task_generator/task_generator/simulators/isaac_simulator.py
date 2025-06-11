import os
import time
import typing
import random

import arena_simulation_setup.entities.robot
import attrs
import rclpy
import rclpy.client

# Import dependencies.
from isaacsim_msgs.srv import (
    DeletePrim,
    GetPrimAttributes,
    ImportObstacles,
    ImportUsd,
    MovePrim,
    Pedestrian,
    SpawnWall,
    UrdfToUsd,
    MovePed,
)
from isaacsim_msgs.msg import Person
from task_generator.shared import DynamicObstacle, ModelType, Obstacle, Robot
from task_generator.simulators import BaseSimulator


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
    move_pedestrians:_Service
    delete_all_pedestrians:_Service


class IsaacSimulator(BaseSimulator):

    def init_service_clients(self):
        """
        Initialize all ROS 2 service clients and wait for their availability.
        """
        self._logger.info("Initializing service clients...")

        # Define services with their corresponding client attributes
        self.services = _Services(
            urdf_to_usd=_Service(type_=UrdfToUsd, name="isaac/urdf_to_usd"),
            import_usd=_Service(type_=ImportUsd, name="isaac/import_usd"),
            delete_prim=_Service(type_=DeletePrim, name="isaac/delete_prim"),
            get_prim_attributes=_Service(
                type_=GetPrimAttributes, name="isaac/get_prim_attributes"
            ),
            move_prim=_Service(type_=MovePrim, name="isaac/move_prim"),
            spawn_wall=_Service(type_=SpawnWall, name="isaac/spawn_wall"),
            import_obstacle=_Service(
                type_=ImportObstacles, name="isaac/import_obstacle"
            ),
            import_pedestrians=_Service(
                type_=Pedestrian, name="isaac/spawn_pedestrian"
            ),
            move_pedestrians=_Service(type_=MovePed, name="isaac/move_pedestrians"),
            delete_all_pedestrians = _Service(type_=DeletePrim, name="isaac/delete_all_pedestrians")
        )

        # Initialize and wait for each service client
        for service in self.services:

            # Create the service client
            service.client = self.node.create_client(service.type_, service.name)

            self._logger.info(f'Waiting for service "{service.name}"...')

            # Wait for the service to become available
            timeout_sec = 10.0
            while not service.client.wait_for_service(timeout_sec=timeout_sec):
                self._logger.warning(
                    f'Service "{service.name}" not available after waiting {timeout_sec}s'
                )
                # raise TimeoutError(f'Service "{service_name}" not available')

            self._logger.info(f'Service "{service.name}" is now available.')

        self._logger.info("All service clients initialized and available.")

    def spawn_entity(self, entity):
        self._logger.info(f"Attempting to spawn model: {entity.name}")

        if isinstance(entity, DynamicObstacle):
            return self._spawn_pedestrian(entity)

        if isinstance(entity, Robot):
            return self._spawn_robot(entity)

        assert isinstance(entity, Obstacle)
        return self._spawn_obstacle(entity)

    def move_entity(self, name, position):
        self._logger.info(f"Attempting to move entitiy: {name}")

        self._logger.info(f"position: {position.x,position.y}")
        self._logger.info(f"orientation: {position.orientation}")

        response = self.services.move_prim.client.call_async(
            MovePrim.Request(
                name=name,
                pose=position.to_pose(),
            )
        )
        if response is None:
            raise RuntimeError(f"failed to move entity: service timed out")

        return True

    def delete_entity(self, name):
        self._logger.info(f"Attempting to delete prim named {name}")

        response = self.services.delete_prim.client.call_async(
            DeletePrim.Request(name=name)
        )

        return True

        # TODO
        if response is None:
            raise RuntimeError(f"failed to delete entity: service timed out")

    def spawn_walls(self, walls):
        # return True
        self._logger.info(f"Attempting to spawn walls")

        self.delete_walls()
        time.sleep(0.01)

        for i, wall in enumerate(walls):
            try:
                # print(f"wall {i+1}: {wall}")
                start = [wall.Start.x, wall.Start.y]
                end = [wall.End.x, wall.End.y]
                future = self.services.spawn_wall.client.call_async(
                    SpawnWall.Request(
                        name=f"wall_{i+1}", start=start, end=end, height=wall.height
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
        model_name = random.choice(
            [
                # "F_Business_02",
                # "F_Medical_01",
                # "M_Medical_01",
                # "biped_demo",
                # "female_adult_police_01_new",
                # "female_adult_police_02",
                # "female_adult_police_03_new",
                # "male_adult_construction_01_new",
                # "male_adult_construction_03",
                # "male_adult_construction_05_new",
                # "male_adult_police_04",
                # "original_female_adult_business_02",
                # "original_female_adult_medical_01",
                # "original_female_adult_police_01",
                # "original_female_adult_police_02",
                # "original_female_adult_police_03",
                # "original_male_adult_construction_01",
                # "original_male_adult_construction_02",
                # "original_male_adult_construction_03",
                # "original_male_adult_construction_05",
                # "original_male_adult_medical_01",
                "original_male_adult_police_04",
            ]
        )
        self.services.import_pedestrians.client.call(
            Pedestrian.Request(
                people=[
                    Person(
                        stage_prefix="/Characters/" + pedestrian.name,
                        character_name=model_name,
                        initial_pose=[
                            pedestrian.position.x,
                            pedestrian.position.y,
                            0.0,
                        ],
                        orientation=pedestrian.position.orientation,
                        controller_stats=False,
                    )
                ]
            )
        )

        return True
    
    # def _move_pedestrian(self, name):


    def _delete_all_pedestrians(self, prim_path):
        self._logger.info(f"Attempting to delete prim named {prim_path}")

        response = self.services.delete_all_pedestrians.client.call_async(
            DeletePrim.Request(name=prim_path)
        )

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
            robot_params = arena_simulation_setup.entities.robot.Robot(robot.model.name)

            response = self.services.urdf_to_usd.client.call(
                UrdfToUsd.Request(
                    name=robot.name,
                    urdf_path=os.path.abspath(model.path),
                    robot_model=robot.model.name,
                    no_localization=False,
                    base_frame=robot_params.base_frame,
                    odom_frame=robot_params.odom_frame,
                    pose=robot.position.to_pose(),
                    cmd_vel_topic=self.node.service_namespace(robot.name, "cmd_vel"),
                )
            )
            return True

        # TODO
        raise NotImplementedError(
            f"robot model of type {model.type} can't be spawned by {self.__class__.__name__}"
        )

    def _spawn_obstacle(self, obstacle: Obstacle) -> bool:
        model = obstacle.model.get([ModelType.USD])
        usd_path = os.path.abspath(model.path)
        response = self.services.import_obstacle.client.call_async(
            ImportObstacles.Request(
                name=obstacle.name,
                usd_path=usd_path,
                pose=obstacle.position.to_pose(),
            )
        )
        return True

    def __init__(self, namespace):
        """Initialize IsaacSimulator

        Args:
            namespace: Namespace for the simulator
        """

        self._logger.info(f"Initializing IsaacSimulator with namespace: {namespace}")

        self.init_service_clients()

        self._logger.info(f"Done initializing Isaac Sim")

    def delete_walls(self):
        self.delete_entity("walls")
