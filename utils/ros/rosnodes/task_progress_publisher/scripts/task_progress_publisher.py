import rospy
import os
import requests

from std_msgs.msg import Empty


class TaskProgressPublisher:
    
    def __init__(self):
        self.task_id = rospy.get_param("task_id")
        self.app_token = rospy.get_param("app_token")
        self.app_token_key = rospy.get_param("app_token_key")
        self.base_url = rospy.get_param("base_url")
        self.task_finished_endpoint = rospy.get_param("task_finished_endpoint")
        self.new_best_model_endpoint = rospy.get_param("new_best_model_endpoint")

        rospy.Subscriber("/scenario_finished", Empty, self.finished_task_callback)
        rospy.Subscriber("/training_finished", Empty, self.finished_task_callback)

        rospy.Subscriber("/training/newBestModel", Empty, self.new_best_model_callback)

    def finished_task_callback(self, _):
        self._post_request(self.task_finished_endpoint)

    def new_best_model_callback(self, _):
        self._post_request(self.new_best_model_endpoint)

    def _post_request(self, endpoint):
        requests.post(
            os.path.join(
                self.base_url,
                endpoint
            ),
            json={ "taskId": self.task_id },
            headers={ self.app_token_key: self.app_token }            
        )


if __name__ == "__main__":
    rospy.init_node("task_progress_publisher")

    task_progress_publihser = TaskProgressPublisher()

    while not rospy.is_shutdown():
        rospy.spin()