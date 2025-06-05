#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from people_msgs.msg import People, Person
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
import math


class PedestrianMarkerPublisher(Node):
    """
    Node that converts people_msgs/People to visualization_msgs/MarkerArray
    for better pedestrian visualization in RViz2
    """

    def __init__(self):
        super().__init__('pedestrian_marker_publisher')
        
        # QoS Settings for people topic (matching your HuNav setup)
        people_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS Settings for marker output
        marker_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriber to people topic
        self.people_subscriber = self.create_subscription(
            People,
            '/people',
            self.people_callback,
            people_qos
        )
        
        # Publisher for pedestrian markers
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/pedestrian_markers',
            marker_qos
        )
        
        # Parameters for visualization
        self.declare_parameter('body_height', 1.6)  # Height of pedestrian body
        self.declare_parameter('body_radius', 0.25)  # Radius of pedestrian body
        self.declare_parameter('head_radius', 0.15)  # Radius of pedestrian head
        self.declare_parameter('arrow_length', 0.6)  # Length of direction arrow
        self.declare_parameter('show_labels', True)  # Show name labels
        self.declare_parameter('show_velocity_arrows', True)  # Show velocity arrows
        
        self.get_logger().info('Pedestrian Marker Publisher initialized')
        self.get_logger().info('Subscribing to /people, publishing to /pedestrian_markers')

    def people_callback(self, msg: People):
        """Convert People message to MarkerArray with stylized human figures"""
        
        marker_array = MarkerArray()
        markers = []
        
        # Get parameters
        body_height = self.get_parameter('body_height').value
        body_radius = self.get_parameter('body_radius').value
        head_radius = self.get_parameter('head_radius').value
        arrow_length = self.get_parameter('arrow_length').value
        show_labels = self.get_parameter('show_labels').value
        show_velocity_arrows = self.get_parameter('show_velocity_arrows').value
        
        # Clear existing markers first (important for dynamic number of people)
        delete_marker = Marker()
        delete_marker.header = msg.header
        delete_marker.action = Marker.DELETEALL
        markers.append(delete_marker)
        
        for i, person in enumerate(msg.people):
            # Use hash of real actor name for marker ID (must be int)
            person_id = hash(person.name) % 1000
            
            # Determine colors based on behavior if available
            body_color, head_color = self._get_person_colors(person)
            
            # 1. Body (Cylinder)
            body_marker = self._create_body_marker(person, person_id, body_color, 
                                                 body_height, body_radius, msg.header)
            markers.append(body_marker)
            
            # 2. Head (Sphere)
            head_marker = self._create_head_marker(person, person_id, head_color,
                                                 head_radius, body_height, msg.header)
            markers.append(head_marker)
            
            # 3. Velocity Arrow (if person is moving)
            if show_velocity_arrows:
                velocity_magnitude = math.sqrt(person.velocity.x**2 + person.velocity.y**2)
                if velocity_magnitude > 0.1:  # Only show if moving fast enough
                    arrow_marker = self._create_velocity_arrow(person, person_id, 
                                                             arrow_length, body_height, msg.header)
                    markers.append(arrow_marker)
            
            # 4. Name Label
            if show_labels:
                label_marker = self._create_name_label(person, person_id, body_height, msg.header)
                markers.append(label_marker)
        
        marker_array.markers = markers
        self.marker_publisher.publish(marker_array)

    def _get_person_colors(self, person: Person):
        """Determine colors based on person behavior/tags"""
        # Default colors
        body_color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.8)  # Light blue
        head_color = ColorRGBA(r=1.0, g=0.8, b=0.6, a=1.0)  # Skin tone
        
        # Check for behavior in tags
        if hasattr(person, 'tags') and len(person.tags) >= 3:
            try:
                behavior = int(person.tags[2])  # Third tag is behavior
                if behavior == 0:  # Regular
                    body_color = ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.8)  # Green
                elif behavior == 1:  # Scared
                    body_color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.8)  # Red
                elif behavior == 2:  # Curious
                    body_color = ColorRGBA(r=1.0, g=1.0, b=0.2, a=0.8)  # Yellow
                elif behavior == 3:  # Threatening
                    body_color = ColorRGBA(r=0.8, g=0.2, b=0.8, a=0.8)  # Purple
            except (ValueError, IndexError):
                pass  # Use default colors
                
        return body_color, head_color

    def _create_body_marker(self, person: Person, person_id: int, color: ColorRGBA,
                           height: float, radius: float, header) -> Marker:
        """Create cylinder marker for person body"""
        marker = Marker()
        marker.header = header
        marker.ns = "pedestrian_bodies"
        marker.id = person_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position (cylinder center is at middle height)
        # FIX: Force Z to ground level (0.0) + half height
        marker.pose.position.x = person.position.x
        marker.pose.position.y = person.position.y
        marker.pose.position.z = 0.0 + height/2  # Always start from ground level
        marker.pose.orientation.w = 1.0
        
        # Size
        marker.scale = Vector3(x=radius*2, y=radius*2, z=height)
        
        # Color
        marker.color = color
        
        # Lifetime
        marker.lifetime.sec = 1  # Disappear after 1 second if not updated
        
        return marker

    def _create_head_marker(self, person: Person, person_id: int, color: ColorRGBA,
                           radius: float, body_height: float, header) -> Marker:
        """Create sphere marker for person head"""
        marker = Marker()
        marker.header = header
        marker.ns = "pedestrian_heads"
        marker.id = person_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position (on top of body)
        # FIX: Force Z to ground level + body height + head radius
        marker.pose.position.x = person.position.x
        marker.pose.position.y = person.position.y
        marker.pose.position.z = 0.0 + body_height + radius  # Ground + body + head
        marker.pose.orientation.w = 1.0
        
        # Size
        marker.scale = Vector3(x=radius*2, y=radius*2, z=radius*2)
        
        # Color
        marker.color = color
        
        # Lifetime
        marker.lifetime.sec = 1
        
        return marker

    def _create_velocity_arrow(self, person: Person, person_id: int, 
                              arrow_length: float, body_height: float, header) -> Marker:
        """Create arrow marker showing velocity direction"""
        marker = Marker()
        marker.header = header
        marker.ns = "pedestrian_arrows"
        marker.id = person_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position at person center
        marker.pose.position.x = person.position.x
        marker.pose.position.y = person.position.y
        marker.pose.position.z = 0.0 + body_height/2  # Ground + half body
        
        # Orientation based on velocity
        velocity_yaw = math.atan2(person.velocity.y, person.velocity.x)
        marker.pose.orientation.z = math.sin(velocity_yaw/2)
        marker.pose.orientation.w = math.cos(velocity_yaw/2)
        
        # Size (arrow length proportional to speed)
        velocity_magnitude = math.sqrt(person.velocity.x**2 + person.velocity.y**2)
        actual_length = min(arrow_length * velocity_magnitude, arrow_length)
        marker.scale = Vector3(x=actual_length, y=0.1, z=0.1)
        
        # Color (bright for visibility)
        marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # Orange
        
        # Lifetime
        marker.lifetime.sec = 1
        
        return marker

    def _create_name_label(self, person: Person, person_id: int, 
                          body_height: float, header) -> Marker:
        """Create text marker with person name"""
        marker = Marker()
        marker.header = header
        marker.ns = "pedestrian_labels"
        marker.id = person_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position above head
        marker.pose.position.x = person.position.x
        marker.pose.position.y = person.position.y
        marker.pose.position.z = 0.0 + body_height + 0.5  # Ground + body + label offset
        marker.pose.orientation.w = 1.0
        
        # Text content - use real actor name
        marker.text = f"Agent {person.name}"
        
        # Size
        marker.scale.z = 0.3  # Text height
        
        # Color (white for visibility)
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        
        # Lifetime
        marker.lifetime.sec = 1
        
        return marker


def main(args=None):
    rclpy.init(args=args)
    
    node = PedestrianMarkerPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down pedestrian marker publisher')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()