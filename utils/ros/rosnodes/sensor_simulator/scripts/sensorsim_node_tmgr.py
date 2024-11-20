#!/usr/bin/env python3
import copy

import pedsim_msgs.msg as peds
import rosnode
import rospy
from ford_msgs.msg import Clusters
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry

# col
from scenario_police import police
from std_msgs.msg import ColorRGBA, Int16
from visualization_msgs.msg import MarkerArray


class sensor:
    def __init__(self):
        # tmgr
        # last updated topic
        self.update_cluster = True
        self.n_reset = -1
        self.obstacles = {}
        self.cluster = Clusters()
        # pub
        self.pub_obst_odom = rospy.Publisher("/obst_odom", Clusters, queue_size=1)
        self.pub_timer = rospy.Timer(rospy.Duration(0.1), self.pub_odom)
        # sub
        self.sub_reset = rospy.Subscriber("/scenario_reset", Int16, self.cb_reset)
        self.cb_reset(0)

    def cb_reset(self, msg):
        # collect static and dynamic obstacles
        self.n_reset += 1
        self.obst_topics = []
        self.get_obstacle_topics()

    def update_obstacle_odom(self):
        # subscriber
        # debug objects have unique topics
        for topic in self.obst_topics:
            rospy.Subscriber(topic, MarkerArray, self.cb_marker, topic)
        # pedsim agents are all collected in one topic
        rospy.Subscriber(
            "/pedsim_simulator/simulated_agents", peds.AgentStates, self.cb_marker
        )

    def get_obstacle_topics(self):
        topics = rospy.get_published_topics()
        for t_list in topics:
            for t in t_list:
                if "/debug/model/obs_" in t:
                    self.obst_topics.append(t)
        self.update_obstacle_odom()
        # print("obstacles:", len(self.obst_topics))

    def pub_odom(self, event):
        self.update_cluster = False
        self.fill_cluster()
        self.pub_obst_odom.publish(self.cluster)
        # reset cluster
        self.cluster = Clusters()
        self.update_cluster = True

    def fill_cluster(self):

        for i, topic in enumerate(self.obstacles):
            tmp_point = Point()
            tmp_point.x = self.obstacles[topic][0].x
            tmp_point.y = self.obstacles[topic][0].y
            tmp_point.z = self.obstacles[topic][1]

            tmp_vel = self.obstacles[topic][2]

            self.cluster.mean_points.append(tmp_point)
            self.cluster.velocities.append(tmp_vel)
            self.cluster.labels.append(i)

    def cb_marker(self, msg, topic=None):

        if self.update_cluster:

            if type(msg) == MarkerArray:
                v = Vector3()
                m = msg.markers[0]
                pos = m.pose.position
                r = m.scale.x / 2
                label = 0
                if topic in self.obstacles:
                    old_pos = self.obstacles[topic][0]
                    old_time = self.obstacles[topic][3].nsecs
                    curr_time = m.header.stamp.nsecs
                    dt = (curr_time - old_time) * 10**-9
                    if dt > 0:
                        v.x = round((pos.x - old_pos.x) / dt, 3)
                        v.y = round((pos.y - old_pos.y) / dt, 3)
                    label = len(self.obst_topics)
                self.obstacles[topic] = [pos, r, v, m.header.stamp, label]
            else:  # Process pedsim agents
                for agent in msg.agent_states:
                    v = agent.twist.linear
                    pos = agent.pose.position
                    label = agent.id
                    self.obstacles[label] = [
                        pos,
                        0.35,
                        v,
                        agent.header.stamp,
                        label,
                    ]


def run():
    rospy.init_node("tb3_sensor_sim", anonymous=False)
    sensor()
    # police()
    rospy.spin()


if __name__ == "__main__":
    run()