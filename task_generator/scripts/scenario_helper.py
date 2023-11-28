#! /usr/bin/env python3

import rospy
import nav_msgs.srv as nav_msgs_srv
import numpy as np

from matplotlib import pyplot as plt


def on_click(event, map):
    x = event.xdata
    y = event.ydata

    origin = map.info.origin.position
    resolution = map.info.resolution

    real_x = x * resolution + origin.x
    real_y = y * resolution + origin.y

    print(f"Point: [{real_x}, {real_y}, 0]")


if __name__ == "__main__":
    rospy.init_node("task_generator")

    rospy.wait_for_service("/static_map")

    service_client_get_map = rospy.ServiceProxy("/static_map", nav_msgs_srv.GetMap)

    map = service_client_get_map().map
    map.data = np.array(map.data)

    map.data[map.data < 0] = 100

    fig = plt.figure()

    imgplot = plt.imshow(np.reshape(
        map.data,
        (map.info.height, map.info.width)
    ))

    fig.canvas.mpl_connect("button_press_event", lambda x: on_click(x, map))

    plt.show()

    # imgplot.show()

    rospy.spin()
