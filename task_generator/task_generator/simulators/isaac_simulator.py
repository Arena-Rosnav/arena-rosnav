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

class IsaacSimulator(BaseSimulator):
    def __init__(self, namespace):
        """Initialize GazeboSimulator

        Args:
            namespace: Namespace for the simulator
        """

        super().__init__(namespace=namespace)
        raise NotImplementedError()
