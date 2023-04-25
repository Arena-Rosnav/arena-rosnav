from task_generator.constants import Constants

from .map_manager import MapManager
from ..simulators.base_simulator import BaseSimulator


class ObstacleManager:
    def __init__(
        self,
        namespace: str,
        map_manager: MapManager,
        simulator: BaseSimulator,
        remove_obstacles=True,
    ):
        self.map_manager = map_manager
        self.namespace = namespace
        self.simulator = simulator
        self.remove_obstacles = remove_obstacles

    def start_scenario(self, scenario):
        self.simulator.spawn_pedsim_agents(scenario["obstacles"]["dynamic"])

    def reset_scenario(self, scenario):
        self.simulator.reset_pedsim_agents()

        self.simulator.remove_all_obstacles()

        print(scenario.get("obstacles").get("static"))

        if not scenario.get("obstacles") or not scenario.get("obstacles").get("static"):
            return

        for obstacle in scenario["obstacles"]["static"]:
            self.simulator.spawn_obstacle(
                position=[*obstacle["pos"], 0],
                yaml_path=obstacle["yaml_path"],
                is_dynamic=False,
            )

    def reset_random(
        self,
        dynamic_obstacles=Constants.ObstacleManager.DYNAMIC_OBSTACLES,
        static_obstacles=Constants.ObstacleManager.STATIC_OBSTACLES,
        forbidden_zones=None,
    ):
        if forbidden_zones is None:
            forbidden_zones = []

        if self.remove_obstacles:
            self.reset_remove_all(
                dynamic_obstacles=dynamic_obstacles,
                static_obstacles=static_obstacles,
                forbidden_zones=forbidden_zones,
            )
        else:
            self.reset_move_all(
                dynamic_obstacles=dynamic_obstacles,
                static_obstacles=static_obstacles,
                forbidden_zones=forbidden_zones,
            )

    def reset_remove_all(
        self,
        dynamic_obstacles=Constants.ObstacleManager.DYNAMIC_OBSTACLES,
        static_obstacles=Constants.ObstacleManager.STATIC_OBSTACLES,
        forbidden_zones=None,
    ):
        if forbidden_zones is None:
            forbidden_zones = []

        self.simulator.remove_all_obstacles()

        obstacles = []

        for _ in range(dynamic_obstacles):
            position = self.map_manager.get_random_pos_on_map(
                safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
                forbidden_zones=forbidden_zones,
            )
            obstacles.append(self.simulator.create_dynamic_obstacle(position=position))

        for _ in range(static_obstacles):
            position = self.map_manager.get_random_pos_on_map(
                safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
                forbidden_zones=forbidden_zones,
            )
            obstacles.append(self.simulator.create_static_obstacle(position=position))

        self.simulator.spawn_obstacles(obstacles)

    def reset_move_all(
        self,
        dynamic_obstacles=Constants.ObstacleManager.DYNAMIC_OBSTACLES,
        static_obstacles=Constants.ObstacleManager.STATIC_OBSTACLES,
        forbidden_zones=None,
    ):
        obstacles = []
        # spawn new obstacles if necessary
        while self.simulator.dynamic_obs_amount < dynamic_obstacles:
            position = self.map_manager.get_random_pos_on_map(
                safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
                forbidden_zones=forbidden_zones,
            )
            obstacles.append(self.simulator.create_dynamic_obstacle(position=position))

        while self.simulator.static_obs_amount < static_obstacles:
            position = self.map_manager.get_random_pos_on_map(
                safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
                forbidden_zones=forbidden_zones,
            )
            obstacles.append(self.simulator.create_static_obstacle(position=position))

        if obstacles:
            self.simulator.spawn_obstacles(obstacles)

        # iterate over bstacles
        for name in self.simulator.obs_names:
            # get random position
            position = self.map_manager.get_random_pos_on_map(
                safe_dist=Constants.ObstacleManager.OBSTACLE_MAX_RADIUS,
                forbidden_zones=forbidden_zones,
            )
            # move obstacle to position
            self.simulator.move_robot(pos=position, name=name)
