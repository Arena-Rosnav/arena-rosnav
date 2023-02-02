import seaborn as sns
import pandas as pd
import rospkg
import os
import numpy as np
import argparse
import matplotlib.pyplot as plt
import json
import yaml

from utils import Utils

"""
    TODO: 
    - Add collisions to path map
"""


## Create plots for only one dataset

aggregate_callbacks = {
    "max": max,
    "min": min,
    "sum": sum,
    "mean": lambda x: sum(x) / len(x)
}

DIST_PLOTS = {
    "strip": sns.stripplot,
    "swarm": sns.swarmplot,
    "box": sns.boxplot,
    "boxen": sns.boxenplot,
    "violin": sns.violinplot
}

CAT_PLOTS = {
    "line": sns.lineplot,
    "bar": sns.barplot
}

SHOULD_SAVE_PLOTS_KEY = "SHOULD_SAVE_PLOTS"
SAVE_PLOTS_LOCATION = "SAVE_PLOTS_LOCATION"

def assert_dist_plot(key):
    assert key in DIST_PLOTS, f"Invalid plot {key} for distribution"

def assert_cat_plot(key):
    assert key in CAT_PLOTS, f"Invalid plot {key} for categorical"

def assert_datakey_valid(key, valid_keys):
    assert key in valid_keys, f"Key {key} not valid"

def plot(title, save_name):
    plt.legend()
    plt.title(title)

    if os.environ.get(SHOULD_SAVE_PLOTS_KEY, "False") == "True":
        plt.savefig(os.path.join(os.environ.get(SAVE_PLOTS_LOCATION), save_name + ".pdf"))
    else:
        plt.show()

    plt.close()

"""

METRIC FILE SCHEMA:

episode                 INT INCREMENT                           # Index of the episode
curvature               FLOAT[]                                 # (N - 2) menger curvatures
normalized_curvature    FLOAT[]                                 # (N - 2) Normalized curvature
roughness               FLOAT[]                                 # (N - 2) Roughness
path_length_values      FLOAT[]                                 # (N - 1) Length of the path the robot travelled in this step
path_length             FLOAT                                   # Complete path length of the episode
acceleration            FLOAT[]                                 # (N - 1) Difference of velocities
jerk                    FLOAT[]                                 # (N - 2) Rate at which the acceleration changes
velocity                FLOAT[][]                               # (N) Real velocity of the robot
cmd_vel                 FLOAT[][]                               # (N) Desired velocity, output of the network
collision_amount        INT                                     # Sum of collisions in this episode
collisions              INT[]                                   # (unknonw) Index of all collisions
path                    FLOAT[][]                               # (N - 1) Positions for each timestep
angle_over_length       FLOAT                                   # Mean change of the angle over the complete path
action_type             (MOVE | STOP | ROTATE)[]                # Action type of the robot for each timestep
time_diff               INT                                     # Time the episode took, ros time in ns
time                    INT[]                                   # Time of each step
result                  TIMEOUT | GOAL_REACHED | COLLISION      # Wether an episode was successful or not

"""

# Read in all metric files
# check if all metrics use the same map 
# concatenate all files in big dataset

def read_datasets(data_paths):

    datasets = []
    scenarios = []

    for path in data_paths:
        base_path = os.path.join("data", path)
        metrics = os.path.join(base_path, "metrics.csv")
        params = os.path.join(base_path, "params.yaml")

        assert os.path.exists(
            metrics
        ) and os.path.exists(params), "Metrics or params file does not exist"

        with open(params) as file:
            params_content = yaml.safe_load(file)

        scenarios.append(params_content["scenario_file"])

        dataset = pd.read_csv(metrics, converters={
            "path_length_values": Utils.string_to_float_list,
            "curvature": Utils.string_to_float_list,
            "roughness": Utils.string_to_float_list,
            "velocity": Utils.string_to_float_list,
            "jerk": Utils.string_to_float_list,
            "start": Utils.string_to_float_list,
            "goal": Utils.string_to_float_list,
            "time": Utils.string_to_float_list,
            "acceleration": lambda a: json.loads(a),
            "path": lambda a: json.loads(a),
        })

        # Set parameters in dataset coloumns for better differentiation
        dataset["local_planner"] = params_content["local_planner"]
        dataset["agent_name"] = params_content["agent_name"]
        dataset["model"] = params_content["model"]
        dataset["namespace"] = params_content["namespace"]

        datasets.append(dataset)

    # If datasets used different scenario files a comparison makes no sense
    assert len(set(scenarios)) == 1, "Scenario files are not the same"

    return pd.concat(datasets), scenarios[0]


## FOR RESULT

class ResultPlotter:
    @staticmethod
    def countplot_for_result(dataset, hue="namespace", title="Results", save_name="results", plot_args={}):
        """
            Shows the results for every episode in a count plot to compare
            with different robots
        """
        sns.countplot(data=dataset, x="result", hue=hue, **plot_args)

        plot(title, save_name)

    @staticmethod
    def plot_result_from_declaration(dataset, result_declaration):
        if result_declaration == None:
            return

        ResultPlotter.countplot_for_result(
            dataset,
            result_declaration["title"],    
            result_declaration["save_name"],    
            result_declaration.get("plot_args", {}),    
        )

## FOR TIME STEP VALUES -> ARRAYS FOR EACH EPISODE
# curvature, normalized_curvature, roughness, path_length_value, acceleration, velocity




class EpisodeArrayValuePlotter:
    """
        This class is used to plot coloumns, which are arrays for single episodes
        That means, values that are collected in every timestep
        This includes all coloumns listed in POSSIBLE_DATA_KEYS
    """
    POSSIBLE_DATA_KEYS = [
        "curvature",
        "normalized_curvature",
        "roughness",
        "path_length_values",
        "acceleration",
        "jerk",
        "velocity"
    ]

    def lineplot_for_single_episode(dataset, data_key, title, save_name, step_size=5, hue="namespace", episode=None, plot_args={}):
        """
            Creates a lineplot to visualize the values for a single episode.

            Args:
                data_key: str -> Name of the coloumn you want to plot
                step_size: int -> Number of values that should be skipped when plotting to reduce data size
                hue: str -> Name of the coloumn to differentiate
                episode: int | None -> Index of the episode, if none then plot mean of all episodes
                plot_args: dict -> further plot arguments
        """
        assert_datakey_valid(data_key, EpisodeArrayValuePlotter.POSSIBLE_DATA_KEYS)

        local_data = dataset[[data_key, hue, "time", "episode"]]

        if episode != None:
            local_data = local_data[local_data["episode"] == episode]

        local_data = local_data.apply(lambda x: EpisodeArrayValuePlotter.resize_time(x, data_key, step_size), axis=1)

        local_data = local_data.explode([data_key, "time"]).reset_index()

        sns.lineplot(data=local_data, y=data_key, x="time", hue=hue, **plot_args)

        plot(title, save_name)

    def distplot_for_single_episode(dataset, data_key, title, save_name, episode=0, plot_key="swarm", plot_args={}):
        """
            Create a distributional plot for a single episode
        """
        assert_datakey_valid(data_key, EpisodeArrayValuePlotter.POSSIBLE_DATA_KEYS)

        assert_dist_plot(plot_key)

        local_data = dataset[dataset["episode"] == episode][[data_key, "namespace"]].explode(data_key)

        DIST_PLOTS[plot_key](data=local_data, y=data_key, x="namespace", **plot_args)

        plot(title, save_name)

    def distplot_for_aggregated(dataset, data_key, aggregate_callback, title, save_name, differentiate="namespace", plot_key="swarm", plot_args={}):
        assert_datakey_valid(data_key, EpisodeArrayValuePlotter.POSSIBLE_DATA_KEYS)
        assert_dist_plot(plot_key)

        local_data = dataset[[data_key, differentiate]]

        def aggregate_value(row):
            row[data_key] = aggregate_callback(row[data_key])

            return row

        local_data = local_data.apply(aggregate_value, axis=1)

        DIST_PLOTS[plot_key](data=local_data, y=data_key, x=differentiate, **plot_args)

        plot(title, save_name)

    def lineplot_for_aggregated(dataset, data_key, aggregate_callback, title, save_name, differentiate="namespace", plot_args={}):
        assert_datakey_valid(data_key, EpisodeArrayValuePlotter.POSSIBLE_DATA_KEYS)

        local_data = dataset[[data_key, differentiate, "episode"]].reset_index()

        def aggregate_value(row):
            row[data_key] = aggregate_callback(row[data_key])

            return row

        local_data = local_data.apply(aggregate_value, axis=1)

        sns.lineplot(data=local_data, x="episode", y=data_key, hue=differentiate, **plot_args)

        plot(title, save_name)

    def resize_time(row, key_reference, step_size):
        time_null = row["time"][0]

        row["time"] = list(map(lambda x: int((x - time_null) / 10e8), row["time"]))
        row["time"] = row["time"][0:len(row[key_reference])]

        row["time"] = row["time"][0::step_size]
        row[key_reference] = row[key_reference][0::step_size]

        return row


## FOR DISCRETE VALUES IN EPISODE

class DiscreteValuePlotter:
    POSSIBLE_VALUES = [
        "time_diff",
        "angle_over_length",
        "collision_amount",
        "path_length"
    ]

    def catplot_over_episodes(dataset, data_key, title, save_name, differentiate="namespace", plot_key="line", plot_args={}):
        assert_datakey_valid(data_key, DiscreteValuePlotter.POSSIBLE_VALUES)
        assert_cat_plot(plot_key)

        CAT_PLOTS[plot_key](data=dataset.reset_index(), x="episode", y=data_key, hue=differentiate, **plot_args)

        plot(title, save_name)

    def distplot_over_episodes(dataset, data_key, title, save_name, differentiate="namespace", plot_key="swarm", plot_args={}):
        assert_datakey_valid(data_key, DiscreteValuePlotter.POSSIBLE_VALUES)
        assert_dist_plot(plot_key)

        DIST_PLOTS[plot_key](data=dataset.reset_index(), y=data_key, x=differentiate, **plot_args)

        plot(title, save_name)


## FOR SHOWING PATH THE ROBOT TOOK

class PathVisualizer:

    def __init__(self, scenario):
        self.scenario_file, self.scenario_content = PathVisualizer.read_scenario_file(scenario)

        self.map_name = self.scenario_content["map"]
        self.map_path, self.map_content = PathVisualizer.read_map_file(self.map_name)

        print(self.map_content)

    def create_map_plot(self):
        map_img = plt.imread(os.path.join(self.map_path, self.map_content["image"]))

        fig, ax = plt.subplots()

        ax.imshow(map_img)

        return fig, ax

    def create_episode_plots_for_namespaces(self, dataset, title, save_name, desired_results=[], should_add_obstacles=False, should_add_collisions=False):
        robots_tested = list(set(dataset["namespace"].to_list()))

        plt.close()

        for namespace in robots_tested:
            fig, ax = self.create_map_plot()

            paths_for_namespace = dataset[dataset["namespace"] == namespace][["path", "result", "start", "goal"]]

            path_amount = len(paths_for_namespace.index)

            for i in range(path_amount):
                path = paths_for_namespace["path"][i]
                result = paths_for_namespace["result"][i]

                if len(desired_results) > 0 and not result in desired_results:
                    break 

                path = np.array(list(map(self.ros_to_real_coord, path))).transpose()

                ax.plot(path[0][:-10], path[1][:-10], label="Episode: " + str(i))

            self.add_obstacles_to_plot(should_add_obstacles)

            self.add_start_and_goal_to_plot(ax, paths_for_namespace["start"][0], paths_for_namespace["goal"][0])

            plot(title, save_name)


    def create_best_plots(self, dataset, should_add_obstacles=True, should_add_collisions=False):
        """
            - Only plot the paths were the goal is reached
            - Plot the path that took the least amount of time
        """
        possible_paths = dataset[dataset["result"] == "GOAL_REACHED"]

        namespaces = list(set(possible_paths["namespace"].to_list()))

        fig, ax = self.create_map_plot()

        for namespace in namespaces:
            paths_for_namespace = possible_paths[possible_paths["namespace"] == namespace]
            
            minimal_path = paths_for_namespace[paths_for_namespace["time_diff"] == paths_for_namespace["time_diff"].min()][["path", "start", "goal"]].reset_index()

            print(minimal_path)

            path = np.array(list(map(self.ros_to_real_coord, minimal_path["path"][0]))).transpose()

            ax.plot(path[0][:-10], path[1][:-10], label=namespace)

            self.add_start_and_goal_to_plot(ax, minimal_path["start"][0], minimal_path["goal"][0])
        
        self.add_obstacles_to_plot(ax, should_add_obstacles)

        # TODO CREATE PLOT TITLE

        plt.legend()
        plt.show()
        plt.close()

    def add_obstacles_to_plot(self, ax, should_add_obstacles=False):
        if not should_add_obstacles:
            return

        obstacles = self.scenario_content["obstacles"]

        for o in obstacles["static"]:
            position = o["pos"]
            map_coordinates = self.ros_to_real_coord(position)

            ax.plot(map_coordinates[0], map_coordinates[1], "o--", color="black", ms=15, alpha=0.2)

        for o in obstacles["dynamic"]:
            verts = [self.ros_to_real_coord(waypoint) for waypoint in o["waypoints"]]
            verts.append(verts[0])

            x, y = zip(*verts)

            ax.plot(x, y, "x--", lw=1, color="black", ms=10, alpha=0.4)

    def add_start_and_goal_to_plot(self, ax, start, goal):
        print(start)

        start = self.ros_to_real_coord(start)

        ax.plot(start[0], start[1], "x", label="Start", ms=10, color="green")

        goal = self.ros_to_real_coord(goal)

        ax.plot(goal[0], goal[1], "*", label="Goal", ms=10, color="red")

    @staticmethod
    def read_scenario_file(scenario):
        name = os.path.join(rospkg.RosPack().get_path("task-generator"), "scenarios", scenario)

        with open(name) as file:
            content = yaml.safe_load(file)

        return name, content

    @staticmethod
    def read_map_file(map_name):
        map_path = os.path.join(rospkg.RosPack().get_path("arena-simulation-setup"), "maps", map_name)

        with open(os.path.join(map_path, "map.yaml")) as file:
            content = yaml.safe_load(file)

        return map_path, content

    def ros_to_real_coord(self, coord):
        return [i / self.map_content["resolution"] for i in coord][:2]


def create_plots_from_declaration_file(declaration_file):
    ## Show plots setup

    show_plots = declaration_file["show_plots"]

    if not show_plots:
        os.environ[SHOULD_SAVE_PLOTS_KEY] = "True"
        location = os.path.join("path", declaration_file["save_location"])
        os.environ[SAVE_PLOTS_LOCATION] = location

        try:
            os.mkdir(location)
        except:
            print("Path", location, "cannot be created")

    ## Dataset setup

    dataset, scenario = read_datasets(declaration_file["datasets"])

    ## Plot Result

    ResultPlotter(dataset, declaration_file.get("result", None))
    
    ## Plot time step values

    single_episode_line = declaration_file.get("single_episode_line", [])

    for line in single_episode_line:
        EpisodeArrayValuePlotter.lineplot_for_single_episode(
            dataset, 
            line["data_key"],
            line["title"],
            line["save_name"],
            step_size=line.get("step_size", 5),
            hue=line.get("hue", "namespace"),
            episode=line.get("episode", None),
            plot_args=line.get("plot_args", {})
        )

    single_episode_distribution = declaration_file.get("single_episode_distribution", [])

    for line in single_episode_distribution:
        EpisodeArrayValuePlotter.distplot_for_single_episode(
            dataset,
            line["data_key"],
            line["title"],
            line["save_name"],
            episode=line["episode"],
            plot_key=line.get("plot_key", "swarm"),
            plot_args=line.get("plot_args", {})
        )

    aggregated_distribution = declaration_file.get("aggregated_distribution", [])

    for line in aggregated_distribution:
        EpisodeArrayValuePlotter.distplot_for_aggregated(
            dataset, 
            line["data_key"], 
            aggregate_callbacks[line["aggregate"]],
            line["title"],
            line["save_name"],
            plot_key=line.get("plot_key", "swarm"),
            plot_args=line.get("plot_args", {})
        )

    aggreagted_line = declaration_file.get("aggregated_line", [])

    for line in aggreagted_line:
        EpisodeArrayValuePlotter.lineplot_for_aggregated(
            dataset, 
            line["data_key"], 
            aggregate_callbacks[line["aggregate"]],
            line["title"],
            line["save_name"],
            plot_args=line.get("plot_args", {})
        )

    # Plot episode values

    all_episodes_categorical = declaration_file.get("all_episodes_categorical", [])

    for line in all_episodes_categorical:
        DiscreteValuePlotter.catplot_over_episodes(
            dataset, 
            line["data_key"], 
            line["title"],
            line["save_name"],
            plot_key=line["plot_key"],
            plot_args=line.get("plot_args", {})
        )

    all_episodes_distribution = declaration_file.get("all_episodes_distribution", [])

    for line in all_episodes_distribution:
        DiscreteValuePlotter.distplot_over_episodes(
            dataset, 
            line["data_key"], 
            line["title"],
            line["save_name"],
            plot_key=line["plot_key"],
            plot_args=line.get("plot_args", {})
        )

    # Plot paths

    path_visualizer = PathVisualizer(scenario)

    episode_plots_for_namespaces = declaration_file.get(episode_plots_for_namespaces, None)

    if episode_plots_for_namespaces != None:
        path_visualizer.create_episode_plots_for_namespaces(
            dataset, 
            episode_plots_for_namespaces["title"],
            episode_plots_for_namespaces["save_name"],
            desired_results=episode_plots_for_namespaces["desired_results"],
            should_add_obstacles=episode_plots_for_namespaces.get("should_add_obstacles", False),
            should_add_collisions=episode_plots_for_namespaces.get("should_add_collisions", False),
        )

    create_best_plots = declaration_file.get(create_best_plots, None)

    if create_best_plots != None:
        path_visualizer.create_best_plots(
            dataset, 
            create_best_plots["title"],
            create_best_plots["save_name"],
            should_add_obstacles=create_best_plots.get("should_add_obstacles", False),
            should_add_collisions=create_best_plots.get("should_add_collisions", False),
        )


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("declaration_file")

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    with open(os.path.join("plot_declarations", args.declaration_file)) as file:
        declaration_file = yaml.safe_load(file)

    create_plots_from_declaration_file(declaration_file)