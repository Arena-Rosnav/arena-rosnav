import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
from geometry_msgs.msg import Pose, Point, Quaternion
import random
import math
import xml.etree.ElementTree as ET

class RandomObstacleSpawner(Node):
    def __init__(self):
        super().__init__('random_obstacle_spawner')
        
        # Create a client for the spawn service
        self.spawn_client = self.create_client(SpawnEntity, '/world/default/create')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        # Parameters for spawning
        self.map_bounds = {
            'x_min': -10.0, 'x_max': 10.0,
            'y_min': -10.0, 'y_max': 10.0
        }
        self.min_obstacle_size = 0.3
        self.max_obstacle_size = 1.0
        
    def generate_random_pose(self):
        """Generate a random pose within map bounds"""
        x = random.uniform(self.map_bounds['x_min'], self.map_bounds['x_max'])
        y = random.uniform(self.map_bounds['y_min'], self.map_bounds['y_max'])
        yaw = random.uniform(-math.pi, math.pi)
        
        pose = Pose()
        pose.position = Point(x=x, y=y, z=0.5)  # z=0.5 to ensure obstacle is above ground
        pose.orientation = Quaternion(x=0.0, y=0.0, z=math.sin(yaw/2), w=math.cos(yaw/2))
        return pose

    def create_box_sdf(self, name, size):
        """Create SDF string for a box obstacle"""
        sdf = ET.Element('sdf', version='1.6')
        model = ET.SubElement(sdf, 'model', name=name)
        
        # Static flag
        static = ET.SubElement(model, 'static')
        static.text = 'true'
        
        # Link
        link = ET.SubElement(model, 'link', name='link')
        
        # Collision
        collision = ET.SubElement(link, 'collision', name='collision')
        geometry = ET.SubElement(collision, 'geometry')
        box = ET.SubElement(geometry, 'box')
        box_size = ET.SubElement(box, 'size')
        box_size.text = f"{size} {size} 1.0"
        
        # Visual
        visual = ET.SubElement(link, 'visual', name='visual')
        geometry = ET.SubElement(visual, 'geometry')
        box = ET.SubElement(geometry, 'box')
        box_size = ET.SubElement(box, 'size')
        box_size.text = f"{size} {size} 1.0"
        
        # Material
        material = ET.SubElement(visual, 'material')
        ambient = ET.SubElement(material, 'ambient')
        ambient.text = '0.3 0.3 0.3 1'
        diffuse = ET.SubElement(material, 'diffuse')
        diffuse.text = '0.7 0.7 0.7 1'
        
        return ET.tostring(sdf, encoding='unicode')

    async def spawn_obstacle(self, index):
        """Spawn a single obstacle"""
        size = random.uniform(self.min_obstacle_size, self.max_obstacle_size)
        name = f'obstacle_{index}'
        pose = self.generate_random_pose()
        sdf = self.create_box_sdf(name, size)
        
        request = SpawnEntity.Request()
        request.name = name
        request.xml = sdf
        request.initial_pose = pose
        
        future = self.spawn_client.call_async(request)
        await future
        
        if future.result().success:
            self.get_logger().info(f'Successfully spawned {name}')
        else:
            self.get_logger().error(f'Failed to spawn {name}')

    async def spawn_multiple_obstacles(self, num_obstacles):
        """Spawn multiple obstacles"""
        for i in range(num_obstacles):
            await self.spawn_obstacle(i)

def main(args=None):
    rclpy.init(args=args)
    spawner = RandomObstacleSpawner()
    
    # Spawn 10 random obstacles
    rclpy.spin_until_future_complete(
        spawner,
        spawner.spawn_multiple_obstacles(10)
    )
    
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()