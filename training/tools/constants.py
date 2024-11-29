import rospkg
import rospy
import os

from datetime import datetime as dt


class TRAINING_CONSTANTS(object):
    class PATHS(object):
        MAIN = rospkg.RosPack().get_path("training")
        ROBOT_MODEL = rospy.get_param("robot_model")
        SIMULATION_SETUP = rospkg.RosPack().get_path('simulation_setup')

        CONFIGS = rospkg.RosPack().get_path("arena_bringup") + "/configs"

        def TRAINING_CONFIGS(file_name): return os.path.join(
            TRAINING_CONSTANTS.PATHS.CONFIGS, "training", file_name
        )

        def REWARD_FUNCTIONS(file_name): return os.path.join(
            TRAINING_CONSTANTS.PATHS.CONFIGS,
            "training",
            "reward_functions",
            f"{file_name}.yaml",
        )

        def MODEL(agent_name): return os.path.join(
            rospkg.RosPack().get_path("rosnav"), "agents", agent_name
        )

        def TENSORBOARD(agent_name): return os.path.join(
            TRAINING_CONSTANTS.PATHS.MAIN, "training_logs", "tensorboard", agent_name
        )

        def EVAL(agent_name): return os.path.join(
            TRAINING_CONSTANTS.PATHS.MAIN, "training_logs", "train_eval_log", agent_name
        )

        def ROBOT_SETTING(robot_model): return os.path.join(
            TRAINING_CONSTANTS.PATHS.SIMULATION_SETUP,
            "robot",
            robot_model,
            f"{robot_model}.model.yaml",
        )

        def AGENT_CONFIG(agent_name): return os.path.join(
            rospkg.RosPack().get_path("rosnav"),
            "agents",
            agent_name,
            "training_config.yaml",
        )

        def CURRICULUM(file_name): return os.path.join(
            TRAINING_CONSTANTS.PATHS.CONFIGS,
            "training",
            "training_curriculums",
            file_name,
        )

    @staticmethod
    def generate_agent_name(architecture_name: str):
        START_TIME = dt.now().strftime("%Y_%m_%d__%H_%M_%S")
        robot_model = rospy.get_param("robot_model")
        encoder_name = rospy.get_param("space_encoder", "RobotSpecificEncoder")
        agent_name = f"{robot_model}_{architecture_name}_{START_TIME}"
        return agent_name
