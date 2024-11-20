import rospy
import time

from rosgraph_msgs.msg import Clock

if __name__ == "__main__":
    rospy.init_node("map_server_clock_simulator")

    publisher = rospy.Publisher("/clock_simulation", Clock, queue_size=10)

    while not rospy.is_shutdown():
        curr_time = time.time()
        msg = Clock()
        msg.clock.secs = int(curr_time)
        msg.clock.nsecs = int((curr_time - int(curr_time)) * 10e8)

        publisher.publish(msg)
        time.sleep(1)


