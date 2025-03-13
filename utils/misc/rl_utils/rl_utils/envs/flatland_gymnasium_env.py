from typing import Any, Dict, Optional, Tuple, Type, Union

import gymnasium
import numpy as np
import rospy
from flatland_msgs.msg import StepWorld  # type: ignore
from geometry_msgs.msg import Twist
from rl_utils.utils.type_alias.observation import InformationDict
from rosnav_rl.observations import (
    DoneObservation,
    FullRangeLaserCollector,
    ObservationManager,
    get_required_observation_units,
)
from rosnav_rl.reward.reward_function import RewardFunction
from rosnav_rl.spaces import BaseSpaceManager
from rosnav_rl.states import SimulationStateContainer
from rosnav_rl.utils.rostopic import Namespace
from rosnav_rl.utils.type_aliases import EncodedObservationDict, ObservationDict
from std_srvs.srv import Empty

from task_generator.task_generator_node import TaskGenerator
from task_generator.tasks import Task

from .utils import determine_termination


class FlatlandEnv(gymnasium.Env):
    metadata = {"render_modes": ["human"]}

    def __init__(
        self,
        ns: Union[str, Namespace],
        space_manager: Union[BaseSpaceManager, Dict[str, Any]],
        reward_function: Union[RewardFunction, Dict[str, Any]],
        simulation_state_container: Optional[SimulationStateContainer],
        max_steps_per_episode=100,
        init_by_call: bool = False,
        wait_for_obs: bool = False,
        obs_unit_kwargs=None,
        task_generator_kwargs=None,
        start_ros_node: bool = True,
        *args,
        **kwargs,
    ):
        """Initialize the Flatland Gymnasium Environment.

        This class inherits from a base environment class and sets up a ROS-based simulation
        environment for reinforcement learning in Flatland.

        Args:
            ns (Union[str, Namespace]): Namespace for the simulation environment
            rl_agent (RL_Agent): Reinforcement learning agent instance
            simulation_state_container (Optional[SimulationStateContainer]): Container for simulation state management
            max_steps_per_episode (int, optional): Maximum number of steps per episode. Defaults to 100
            init_by_call (bool, optional): Whether to initialize environment on call. Defaults to False
            wait_for_obs (bool, optional): Whether to wait for observations. Defaults to False
            obs_unit_kwargs (dict, optional): Keyword arguments for observation unit. Defaults to None
            task_generator_kwargs (dict, optional): Keyword arguments for task generator. Defaults to None
            *args: Variable length argument list
            **kwargs: Arbitrary keyword arguments

        Raises:
            ValueError: If in training mode and reward function is not provided

        Attributes:
            ns (Namespace): Namespace object for the simulation
            _debug_mode (bool): Debug mode status from ROS parameter
            _is_train_mode (bool): Training mode status from ROS parameter
            _step_size (float): Step size from ROS parameter
            _steps_curr_episode (int): Current step count in episode
            _episode (int): Current episode number
        """
        super(FlatlandEnv, self).__init__()
        self.ns = Namespace(ns) if type(ns) is str else ns

        self._debug_mode = rospy.get_param("/debug_mode", False)
        self._is_train_mode = rospy.get_param("/train_mode", default=True)
        self._step_size = rospy.get_param("/step_size")

        if self._is_train_mode and reward_function is None:
            raise ValueError("Reward function is required for the training.")

        if not self._debug_mode or start_ros_node:
            rospy.init_node(f"env_{self.ns.simulation_ns}".replace("/", "_"))

        self._initialize_agent_components(
            space_manager=space_manager,
            reward_function=reward_function,
        )
        self.__simulation_state_container = simulation_state_container

        self._obs_unit_kwargs = obs_unit_kwargs if obs_unit_kwargs else {}
        self._task_generator_kwargs = (
            task_generator_kwargs if task_generator_kwargs else {}
        )

        self.__wait_for_obs = wait_for_obs

        self._steps_curr_episode = 0
        self._episode = 0
        self._max_steps_per_episode = max_steps_per_episode
        self.__is_first = True

        if not init_by_call:
            self.init()

    def init(self):
        """
        Initializes the environment for training or evaluation.

        If the environment is in training mode, it sets up the environment accordingly.
        It then determines the required observation units based on the reward function
        and the observation space list. If a full range laser is attached to the robot,
        it adds the FullRangeLaserCollector to the observation units.

        Finally, it initializes the ObservationManager with the required observation units
        and other necessary parameters.

        Attributes:
            is_train_mode (bool): Indicates if the environment is in training mode.
            _setup_env_for_training (function): Sets up the environment for training.
            _reward_function (object): The reward function used in the environment.
            _model_space_manager (object): Manages the observation space list.
            __simulation_state_container (object): Contains the state of the simulation.
            _obs_unit_kwargs (dict): Additional keyword arguments for observation units.
            __wait_for_obs (bool): Indicates if the environment should wait for observations.
            ns (str): Namespace for the observation manager.
        """
        if self.is_train_mode:
            self._setup_env_for_training()

        required_obs_units = get_required_observation_units(
            self._reward_function.reward_units
            + self._model_space_manager.observation_space_list
            if self.is_train_mode
            else self._model_space_manager.observation_space_list
        )

        # get obs structure
        if self.__simulation_state_container.robot.laser_state.attach_full_range_laser:
            required_obs_units.append(FullRangeLaserCollector)

        self.observation_collector = ObservationManager(
            ns=self.ns,
            obs_structur=required_obs_units,
            simulation_state_container=self.__simulation_state_container,
            obs_unit_kwargs=self._obs_unit_kwargs,
            wait_for_obs=self.__wait_for_obs,
        )

    @property
    def action_space(self) -> gymnasium.spaces.Box:
        """
        Returns the action space of the environment.

        Returns:
            action_space (object): The action space of the environment.
        """
        return self._model_space_manager.action_space

    @property
    def observation_space(self) -> gymnasium.spaces.Dict:
        """
        Returns the observation space of the environment.

        Returns:
            gym.Space: The observation space of the environment.
        """
        return self._model_space_manager.observation_space

    @property
    def simulation_state_container(self) -> SimulationStateContainer:
        return self.__simulation_state_container

    def _initialize_agent_components(
        self,
        space_manager: BaseSpaceManager,
        reward_function: RewardFunction,
    ):
        if isinstance(space_manager, BaseSpaceManager):
            self._model_space_manager = space_manager

        if isinstance(reward_function, RewardFunction):
            self._reward_function = reward_function

        if isinstance(space_manager, dict):
            self._model_space_manager = BaseSpaceManager(**space_manager)

        if isinstance(reward_function, dict):
            self._reward_function = RewardFunction(**reward_function)

        assert isinstance(self._model_space_manager, BaseSpaceManager)
        assert isinstance(self._reward_function, RewardFunction)

    def _setup_env_for_training(self):
        """
        Sets up the environment for training by initializing necessary components.

        This method performs the following tasks:
        1. Instantiates the task manager using the TaskGenerator class.
        2. Retrieves a predefined task and assigns it to the `self.task` attribute.
        3. Sets up the agent action publisher to publish Twist messages to the 'cmd_vel' topic.
        4. Configures the step world service and publisher for stepping the simulation world.

        Attributes:
            task (Type[Task]): The predefined task for the environment.
            agent_action_pub (rospy.Publisher): Publisher for agent actions.
            _service_name_step (str): Name of the step world service.
            _step_world_publisher (rospy.Publisher): Publisher for stepping the simulation world.
            _step_world_srv (rospy.ServiceProxy): Service proxy for the step world service.
        """
        # instantiate task manager
        task_generator = TaskGenerator(
            namespace=self.ns.simulation_ns,
            task_state=self.__simulation_state_container.task,
            robot_state=self.__simulation_state_container.robot,
        )
        self.task: Type[Task] = task_generator._get_predefined_task(
            **self._task_generator_kwargs
        )

        # agent action publisher
        self.agent_action_pub = rospy.Publisher(
            str(self.ns("cmd_vel")), Twist, queue_size=1
        )

        # step world service and publisher
        self._service_name_step = str(self.ns.simulation_ns("step_world"))
        self._step_world_publisher = rospy.Publisher(
            self._service_name_step, StepWorld, queue_size=10
        )
        self._step_world_srv = rospy.ServiceProxy(
            self._service_name_step, Empty, persistent=True
        )

    def _pub_action(self, action: np.ndarray):
        """
        Publishes the given action to the agent's action topic.

        Args:
            action (np.ndarray): The action to be published. It should be a 1D numpy array of length 3,
                                 representing the linear x, linear y, and angular z components of the action.

        Raises:
            AssertionError: If the length of the action array is not 3.
        """
        assert len(action) == 3

        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.linear.y = action[1]
        action_msg.angular.z = action[2]

        self.agent_action_pub.publish(action_msg)

    def _decode_action(self, action: np.ndarray) -> np.ndarray:
        """
        Decodes the given action using the model space encoder.

        Args:
            action (np.ndarray): The action to be decoded.

        Returns:
            np.ndarray: The decoded action.
        """
        return self._model_space_manager.decode_action(action)

    def _encode_observation(
        self, observation: ObservationDict, *args, **kwargs
    ) -> EncodedObservationDict:
        """
        Encodes the given observation using the model space encoder.

        Args:
            observation (ObservationDict): The observation to be encoded.

        Returns:
            The encoded observation.

        """
        return self._model_space_manager.encode_observation(observation, **kwargs)

    def step(
        self, action: np.ndarray
    ) -> Tuple[EncodedObservationDict, float, bool, bool, InformationDict]:
        """
        Execute one step in the environment using the given action.

        Args:
            action (np.ndarray): The action to be taken in the environment.

        Returns:
            Tuple[EncodedObservationDict, float, bool, bool, dict]: A tuple containing:
                - EncodedObservationDict: The encoded observation dictionary after applying the model space manager.
                - float: The reward obtained after taking the action.
                - bool: A flag indicating if the episode has ended.
                - bool: A flag indicating if the episode was truncated (always False in this implementation).
                - InformationDict: Additional information about the step.
        """
        self._pub_action(self._decode_action(action))

        if self.is_train_mode:
            self._call_service_takeSimStep()

        obs_dict: ObservationDict = self.observation_collector.get_observations(
            simulation_state_container=self.__simulation_state_container,
            is_first=self.__is_first,
        )

        # calculate reward
        reward, reward_info = self._reward_function.get_reward(
            obs_dict=obs_dict,
            simulation_state_container=self.__simulation_state_container,
        )

        self._steps_curr_episode += 1

        # info
        info, done = determine_termination(
            reward_info=reward_info,
            curr_steps=self._steps_curr_episode,
            max_steps=self._max_steps_per_episode,
        )

        obs_dict.update({"is_terminal": done})
        self.__is_first = False

        return (
            self._encode_observation(obs_dict, is_done=done),
            reward,
            done,
            False,
            info,
        )

    def reset(
        self, seed=None, options=None
    ) -> Tuple[EncodedObservationDict, InformationDict]:
        """
        Resets the environment to an initial state and returns an initial observation.

        Args:
            seed (int, optional): The seed for random number generation. Defaults to None.
            options (dict, optional): Additional options for the reset. Defaults to None.

        Returns:
            Tuple[EncodedObservationDict, InformationDict]:
                A tuple containing the encoded observation dictionary and an information dictionary.
        """
        super().reset(seed=seed)
        self._episode += 1

        self._before_task_reset()

        first_map = self._episode <= 1 if "sim_1" in self.ns else False

        self.task.reset(
            first_map=first_map,
            reset_after_new_map=self._steps_curr_episode == 0,
        )
        self._reward_function.reset()
        self._steps_curr_episode = 0

        self._after_task_reset()

        obs_dict = self.observation_collector.get_observations(
            is_terminal=False, is_first=True
        )
        obs_dict.update({DoneObservation.name: True})

        self.__is_first = True
        return (
            self._encode_observation(obs_dict),
            dict(),
        )

    def close(self):
        """
        Close the environment.

        """
        rospy.signal_shutdown("Closing environment...")

    def _before_task_reset(self):
        """
        Perform any necessary steps before resetting the task.

        """
        # make sure all simulation components are ready before first episode
        if self._episode <= 1:
            for _ in range(4):
                self.agent_action_pub.publish(Twist())
                self._call_service_takeSimStep()

    def _after_task_reset(self):
        """
        Perform any necessary steps after resetting the task.

        """
        if self.is_train_mode:
            # extra step for planning serivce to provide global plan
            for _ in range(4):
                self.agent_action_pub.publish(Twist())
                self._call_service_takeSimStep()

    def _call_service_takeSimStep(self, t: float = None, srv_call: bool = True):
        """
        Calls the service to take a simulation step in the Flatland environment.

        Args:
            t (float, optional): The time duration for the simulation step. If None,
                                 the default step size is used. Defaults to None.
            srv_call (bool, optional): Flag to determine whether to call the service
                                       to step the world. Defaults to True.

        """
        if srv_call:
            self._step_world_srv()
            return

        request = StepWorld()
        request.required_time = self._step_size if t is None else t

        self._step_world_publisher.publish(request)

    @property
    def is_train_mode(self) -> bool:
        return self._is_train_mode
