# ObstacleInterface


import dataclasses
import math
from typing import Dict, List, Optional
from task_generator.constants import Config
from task_generator.shared import DynamicObstacle, ModelWrapper, Obstacle, PositionOrientation, PositionRadius
from task_generator.tasks import Props_


class ITF_Obstacle:
    """
    Helper methods to fill partially initialized obstacles
    """

    @classmethod
    def create_dynamic_obstacle(
        cls,
        props: Props_,
        waypoints: Optional[List[PositionRadius]] = None,
        n_waypoints: int = 2,
        **kwargs
    ) -> DynamicObstacle:
        """
        Create dynamic obstacle from partial params.
        @name: Name of the obstacle
        @model: ModelWrapper of models
        @position: (optional) Starting position
        @waypoints: (optional) List of waypoints
        @extra: (optional) Extra properties to store
        """

        setup = cls.create_obstacle(props, **kwargs)

        if waypoints is None:

            waypoints = [PositionRadius(setup.position.x, setup.position.y, 1)]
            safe_distance = 0.1  # the other waypoints don't need to avoid robot

            waypoints += [PositionRadius(*pos, 1) for pos in props.world_manager.get_positions_on_map(n=n_waypoints, safe_dist=safe_distance)]

        return DynamicObstacle(**{
            **dataclasses.asdict(setup),
            **dict(waypoints=waypoints)
        })


    @classmethod
    def create_obstacle(
        cls,
        props: Props_,
        name: str,
        model: ModelWrapper,
        position: Optional[PositionOrientation] = None,
        extra: Optional[Dict] = None,
        **kwargs,
    ) -> Obstacle:
        """
        Create non-dynamic obstacle from partial params.
        @name: Name of the obstacle
        @model: ModelWrapper of models
        @position: (optional) Starting position
        @extra: (optional) Extra properties to store
        """

        safe_distance = 1

        if position is None:
            point = props.world_manager.get_position_on_map(safe_distance)
            position = PositionOrientation(point.x, point.y, Config.General.RNG.random() * 2*math.pi)

        if extra is None:
            extra = dict()

        return Obstacle(
            position=position, name=name, model=model, extra=extra, **kwargs
        )

