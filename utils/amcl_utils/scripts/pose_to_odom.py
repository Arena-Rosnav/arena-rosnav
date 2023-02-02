import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class PoseToOdom:
    def __init__(self):
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.transform_pose)
        self.odom_pub = rospy.Publisher("odom_amcl", Odometry, queue_size=10)

        self.seq = 0

    def transform_pose(self, data):
        pose = data.pose

        new_odom_msg = Odometry()

        header = Header()
        header.seq = self.seq
        header.stamp = rospy.get_rostime()
        header.frame_id = "burger_0_0/odom"

        new_odom_msg.child_frame_id = "burger_0_0/base_footprint"
        new_odom_msg.pose = pose
        
        self.odom_pub.publish(new_odom_msg)

        self.seq += 1


if __name__ == "__main__":
    rospy.init_node("pose_to_odom", anonymous=True)

    pose_to_odom = PoseToOdom()

    print("SETUP FINISHED")

    rospy.spin()
