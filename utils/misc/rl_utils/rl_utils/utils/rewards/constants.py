class REWARD_CONSTANTS:
    MIN_GOAL_RADIUS = 0.1
    NO_MOVEMENT_TOLERANCE = 0.1


class DEFAULTS:
    class GOAL_REACHED:
        REWARD: float = 15.0
        _ON_SAFE_DIST_VIOLATION: bool = True

    class SAFE_DISTANCE:
        REWARD: float = -0.15

    class NO_MOVEMENT:
        REWARD: float = -0.01
        _ON_SAFE_DIST_VIOLATION: bool = True

    class APPROACH_GOAL:
        POS_FACTOR: float = 0.3
        NEG_FACTOR: float = 0.5
        _GOAL_UPDATE_THRESHOLD: float = 2.5
        _TERM_THRESHOLD: float = 2.0
        _ON_SAFE_DIST_VIOLATION: bool = True

    class COLLISION:
        REWARD: float = -10.0
        BUMPER_ZONE: float = 0.05

    class DISTANCE_TRAVELLED:
        CONSUMPTION_FACTOR: float = 0.005
        LIN_VEL_SCALAR: float = 1.0
        ANG_VEL_SCALAR: float = 0.001
        _ON_SAFE_DIST_VIOLATION: bool = True

    class APPROACH_GLOBALPLAN:
        POS_FACTOR: float = 0.3
        NEG_FACTOR: float = 0.5
        _ON_SAFE_DIST_VIOLATION: bool = True

    class FOLLOW_GLOBALPLAN:
        MIN_DIST_TO_PATH: float = 0.5
        REWARD_FACTOR: float = 0.1
        _ON_SAFE_DIST_VIOLATION: bool = True

    class REVERSE_DRIVE:
        REWARD: float = 0.01
        THRESHOLD: float = -0.5
        _ON_SAFE_DIST_VIOLATION: bool = True

    class ABRUPT_VEL_CHANGE:
        VEL_FACTORS: dict = {"0": 0.3, "1": 0.07, "2": 0.07}
        _ON_SAFE_DIST_VIOLATION: bool = True

    class ROOT_VEL_DIFF:
        K = 500
        _ON_SAFE_DIST_VIOLATION: bool = False

    class TWO_FACTOR_VEL_DIFF:
        ALPHA = 0.01
        BETA = 0.025

    class PED_SAFE_DISTANCE:
        REWARD: float = -0.14
        SAFE_DIST: float = 0.51

    class OBS_SAFE_DISTANCE:
        REWARD: float = -0.14
        SAFE_DIST: float = 0.21

    class PED_TYPE_SPECIFIC_SAFETY_DISTANCE:
        TYPE: int = 1
        REWARD: float = -0.25
        DISTANCE: float = 1.25
        _ON_SAFE_DIST_VIOLATION: bool = True

    class PED_TYPE_SPECIFIC_COLLISION:
        TYPE: int = 1
        REWARD: float = -10.0
        BUMPER_ZONE: float = 0.05

    class PED_TYPE_SPECIFICE_FACTORED_SAFETY_DISTANCE:
        TYPE: int = 1
        DISTANCE: float = 1.25
        FACTOR: float = -0.5
        _ON_SAFE_DIST_VIOLATION: bool = True
