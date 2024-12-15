import rclpy.client
from task_generator.simulators import BaseSimulator
# Use the isaacsim to import SimulationApp
from isaacsim import SimulationApp

# Setting the config for simulation and make an simulation.
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)

# Import dependencies.
import carb
import math
import omni
import omni.graph.core as og
import usdrt.Sdf
import numpy as np
import matplotlib.pyplot as plt
import yaml
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.rotations import quat_to_euler_angles
from omni.isaac.core.utils import extensions, stage, nucleus
from omni.isaac.nucleus import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.utils.prims import delete_prim,get_prim_at_path,set_prim_attribute_value,get_prim_attribute_value,get_prim_attribute_names
from omni.isaac.core.world import World
from omni.importer.urdf import _urdf
from omni.isaac.sensor import Camera, LidarRtx
from omni.isaac.range_sensor import _range_sensor
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
import omni.isaac.core.utils.numpy.rotations as rot_utils
from pxr import Gf, Usd, UsdGeom
import omni.kit.commands as commands
import numpy as np
import rclpy
from rclpy.node import Node
from isaacsim_msgs.msg import Euler, Quat, Env, Values
from isaacsim_msgs.srv import ImportUsd, ImportUrdf, UrdfToUsd, DeletePrim, GetPrimAttributes, MovePrim, ImportYaml, ScalePrim, SpawnWall
from sensor_msgs.msg import JointState

from task_generator.shared import ModelType, Namespace, PositionOrientation, RobotProps
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import launch
import launch_ros
import ament_index_python

class IsaacSimulator(BaseSimulator):
    
    def init_service_clients(self):
        """Initialize all ROS 2 service clients."""
        self.urdf_to_usd_client = self.node.create_client(
            UrdfToUsd,
            'isaac/urdf_to_usd',
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        
        self.spawn_entity_client = self.node.create_client(
            ImportUsd,
            'isaac/import_usd',
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.delete_entity_client = self.node.create_client(
            DeletePrim,
            'isaac/delete_prim',
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.get_entity_attributes_client = self.node.create_client(
            GetPrimAttributes,
            'isaac/get_prim_attributes',
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.move_entity_client = self.node.create_client(
            MovePrim,
            'isaac/move_prim',
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.spawn_wall_client = self.node.create_client(
            SpawnWall,
            'isaac/spawn_wall',
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Wait for all services to be available
        self.wait_for_services()
    
    def spawn_entity(self, entity):
        self.node.get_logger().info(
            f"Attempting to spawn entitiy: {entity.name}"
        )
        
        try:
            model = entity.model.get(
                [ModelType.USD, ModelType.URDF, ModelType.SDF])
        except FileNotFoundError as e:
            self.node.get_logger().error(repr(e))
            return True
        
        model = entity.model.get([ModelType.USD, ModelType.URDF, ModelType.SDF])
        
        if model.type == ModelType.SDF:
            return True #TODO
        
        description = model.description

        if model.type == ModelType.URDF:
            usd_path = self.urdf_to_usd_client.call(
                UrdfToUsd.Request(
                    name = entity.name,
                    urdf_path = model.path
                )
            ).usd_path

        else:
            usd_path = model.path
            
        response = self.spawn_entity_client.call(
            ImportUsd.Request(
                name = entity.name,
                usd_path = usd_path,
                prim_path = "/World"                
            )
        )
        if response is None:
            raise RuntimeError(
                f'failed to spawn entity: service timed out')

        if response.result > 0:
            raise RuntimeError(
                f'failed to spawn entity: status code {response.result}')

        return True
    
    def move_entity(self,name,position,orientation):
        self.node.get_logger().info(
            f"Attempting to move entitiy: {name}"
        )
        prim_path = f"/World/{name}"
        
        response = self.spawn_entity_client.call(
            MovePrim.Request(
                name = name.name,
                prim_path = prim_path,
                values = [
                    Values(values=position),
                    Values(values=orientation)]               
            )
        )
        if response is None:
            raise RuntimeError(
                f'failed to move entity: service timed out')

        if response.result > 0:
            raise RuntimeError(
                f'failed to move entity: status code {response.result}')

        return True
    
    def delete_entity(self,name):
        self.node.get_logger().info(
            f"Attempting to delete prim named {name}"
        )
        
        prim_path = f"/World/{name}"

        response = self.spawn_entity_client.call(
            MovePrim.Request(
                name = name.name,
                prim_path = prim_path         
            )
        )
        if response is None:
            raise RuntimeError(
                f'failed to delete entity: service timed out')

        if response.result > 0:
            raise RuntimeError(
                f'failed to delete entity: status code {response.result}')

        return True

    def spawn_wall(self,walls):
        self.node.get_logger().info(
            f"Attempting to spawn walls"
        )
        
        world_path = "/World"
        
        for i,wall in enumerate(walls):
            start = [wall.Start.x,wall.Start.y]
            end = [wall.End.x,wall.End.y]
            response = self.spawn_entity_client.call(
            SpawnWall.Request(
                name = f"wall_{i+1}",
                world_path = world_path,
                start = start,
                end = end,
                height = wall.height   
            )
        )
            if response is None:
                raise RuntimeError(
                    f'failed to spawn wall number {i}: service timed out')

            if response.result > 0:
                raise RuntimeError(
                    f'failed to spawn wall number {i}: status code {response.result}')

        return True
            
    def __init__(self, namespace):
        """Initialize IsaacSimulator

        Args:
            namespace: Namespace for the simulator
        """

        self.node.get_logger().info(
            f"Initializing IsaacSimulator with namespace: {namespace}")
        
        self.init_service_clients()
        