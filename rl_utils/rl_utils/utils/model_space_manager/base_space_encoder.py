class BaseSpaceEncoder:
    def __init__(self, laser_num_beams, laser_max_range, radius, is_holonomic, actions, is_action_space_discrete):
        self._laser_num_beams = laser_num_beams
        self._laser_max_range = laser_max_range
        self._radius = radius
        self._is_holonomic = is_holonomic
        self._actions = actions
        self._is_action_space_discrete = is_action_space_discrete

    def get_observation_space(self):
        raise NotImplementedError()

    def get_action_space(self):
        raise NotImplementedError()

    def decode_action(self, action):
        raise NotImplementedError()

    def encode_observation(self, observation):
        raise NotImplementedError()