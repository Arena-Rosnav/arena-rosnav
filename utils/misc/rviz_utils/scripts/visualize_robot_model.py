import rospy
import rospkg
import os
import traceback
import yaml
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3


class VisualizeRobotModel:
    def __init__(self):
        self.srv_start_setup = rospy.Service(
            "start_model_visualization", 
            Empty, 
            self.start_model_visualization_callback
        )

        self.robot_models = {}
        self.publisher_map = {}
        self.subscribers = []

    def start_model_visualization_callback(self, _):
        robot_names = rospy.get_param("robot_names", [])

        robot_odom_topic = VisualizeRobotModel.get_complexity_odom_topic()

        for name in robot_names:
            robot_model = rospy.get_param(os.path.join(name, "robot_model"))

            model_file = VisualizeRobotModel.read_robot_model_file(robot_model)

            markers_for_model = VisualizeRobotModel.create_marker_array_for_robot(
                model_file
            )

            self.robot_models[robot_model] = markers_for_model

            self.publisher_map[name] = rospy.Publisher(
                os.path.join(name, "visualize", "model"),
                MarkerArray,
                queue_size=10
            )

            self.subscribers.append(
                rospy.Subscriber(
                    os.path.join(name, robot_odom_topic), 
                    Odometry, 
                    self.publish_model,
                    (robot_model, name)
                )
            )

        return EmptyResponse()

    def publish_model(self, data, args):
        robot_model, name = args

        try:
            markers = self.robot_models[robot_model]
        except:
            print("Error - Getting markers from dict", robot_model)
            return

        for marker in markers:
            marker.header = data.header
            marker.header.frame_id = "map"
            marker.pose = data.pose.pose

        try:
            self.publisher_map[name].publish(MarkerArray(markers))
        except:
            traceback.print_exc()
            print("Error - publishing markers", name, markers)

    @staticmethod
    def read_robot_model_file(robot_model):
        file_path = os.path.join(
            rospkg.RosPack().get_path("arena_simulation_setup"),
            "entities",
            "robots",
            robot_model,
            f"{robot_model}.model.yaml"
        )

        with open(file_path) as file:
            return yaml.safe_load(file)["bodies"]
    
    @staticmethod
    def create_marker_array_for_robot(bodies):
        markers = []

        for body in bodies:
            color = body["color"]

            for footprint in body["footprints"]:
                marker = VisualizeRobotModel.create_marker_from_footprint(
                    footprint, color
                )

                markers.append(marker)

        return markers

    @staticmethod
    def create_marker_from_footprint(footprint, color):
        type = footprint["type"]
        r, g, b, a = color

        marker = Marker()
        marker.action = Marker.MODIFY
        marker.color = ColorRGBA(r, g, b, a)

        if type == "circle":
            marker.type = Marker.SPHERE
            marker.scale = Vector3(
                footprint["radius"] * 2, 
                footprint["radius"] * 2, 
                0.1
            )

            return marker

        marker.type = Marker.LINE_STRIP
        marker.scale = Vector3(0.03, 0, 0)
        marker.points = [Point(x, y, 0) for x, y in footprint["points"]]
        marker.points.append(marker.points[0])

        return marker

    @staticmethod
    def get_complexity_odom_topic():
        complexity = rospy.get_param("complexity", 1)

        if complexity == 1:
            return "odom"
        if complexity == 2:
            return "odom_amcl"


if __name__ == "__main__":
    rospy.init_node("visualize_robot_model")

    visualizer = VisualizeRobotModel()

    while not rospy.is_shutdown():
        rospy.spin()
