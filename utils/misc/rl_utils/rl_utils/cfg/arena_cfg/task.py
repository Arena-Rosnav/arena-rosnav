from pydantic import BaseModel, model_validator
import rospy


class TaskCfg(BaseModel):
    tm_robots: str = "random"
    tm_obstacles: str = "random"
    tm_modules: str = "staged"

    @model_validator(mode="after")
    def propagate_task_modules(self):
        rospy.set_param("tm_robots", self.tm_robots)
        rospy.set_param("tm_obstacles", self.tm_obstacles)
        rospy.set_param("tm_modules", self.tm_modules)
        return self
