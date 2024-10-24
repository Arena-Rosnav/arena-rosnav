from typing import TYPE_CHECKING, List, Set, Type, Union

from rl_utils.utils.observation_collector import (
    ObservationCollectorUnit,
    ObservationGeneratorUnit,
)

if TYPE_CHECKING:
    from rl_utils.utils.type_alias.observation import (
        ObservationCollector,
        ObservationGenerator,
        ObservationGeneric,
    )
    from rosnav_rl.reward.reward_units.base_reward_units import RewardUnit
    from rosnav_rl.spaces import BaseObservationSpace


def get_required_observation_units(
    list_of_units: Union[List["BaseObservationSpace"], List["RewardUnit"]]
) -> list:
    """
    Collects and returns a list of unique required observation units from the provided list of units.

    Args:
        list_of_units (Union[List["BaseObservationSpace"], List["RewardUnit"]]):
            A list containing instances of either BaseObservationSpace or RewardUnit.

    Returns:
        list: A list of unique required observation units.
    """
    observations = []
    for unit in list_of_units:
        for observation_cls in unit.required_observation_units:
            observations.extend(retrieve_unique_unit(observation_cls, True))
    return list(set(observations))


def retrieve_unique_unit(
    unit: Union["ObservationCollector", "ObservationGenerator"],
    include_generators: bool = False,
) -> Set[Type["ObservationCollector"]]:
    is_collector = issubclass(unit, ObservationCollectorUnit)
    collectors = [unit] if include_generators and not is_collector else []

    if is_collector:
        collectors.append(unit)
        return collectors

    for observations in unit.requires:
        if observations in collectors:
            continue

        if include_generators and issubclass(observations, ObservationGeneratorUnit):
            collectors.append(observations)
        collectors.extend(retrieve_unique_unit(observations, include_generators))
    return set(collectors)


def explore_hierarchy(list_of_units: List["ObservationGeneric"]):
    """
    Traverses the hierarchy of observation units and returns a dictionary
    that represents the depth of each unit in the hierarchy.

    Args:
        list_of_units (List[ObservationGeneric]): A list of observation units.

    Returns:
        dict: A dictionary where the keys are observation units and the values
        are their depths in the hierarchy.
    """

    def traverse_hierarchy(
        unit: "ObservationGeneric", hierarchy_dict: dict, recursion_depth: int = 0
    ) -> int:
        if unit not in hierarchy_dict:
            hierarchy_dict[unit] = 0

        if issubclass(unit, ObservationCollectorUnit):
            pass
        else:
            for observation in unit.requires:
                depth = traverse_hierarchy(
                    observation, hierarchy_dict, recursion_depth + 1
                )
                if depth > hierarchy_dict.get(observation, 0):
                    hierarchy_dict[observation] = depth
        return recursion_depth

    hierarchy_dict = {}

    for unit in list_of_units:
        traverse_hierarchy(unit, hierarchy_dict)

    hierarchy_dict = dict(
        sorted(hierarchy_dict.items(), key=lambda item: item[1], reverse=True)
    )

    return hierarchy_dict


# if __name__ == "__main__":
#     dist = retrieve_unique_unit(DistAngleToGoal, True)
#     social_state = retrieve_unique_unit(PedestrianRelativeLocationGenerator, True)
#     # set out of dist and social_state
#     set_out = set(dist) - set(social_state)
#     print(set_out)

#     c = explore_hierarchy(
#         [
#             PedestrianRelativeVelXGenerator,
#             PedestrianRelativeLocationGenerator,
#             DistAngleToGoal,
#         ]
#     )
