```yaml

# Wether you want to show or save the plots
show_plots: boolean
# Name of the directory in ./path
save_location: string

# List of all datasets that should be compared
# Name of the directory in ./data
datasets: string[]

# Wether you want to plot the result counts
results:
    # Should plot?
    plot: boolean
    # Title of the plot
    title: string
    # Name of the file the plot should be saved ot
    save_name: string
    # Additional Plot arguments
    plot_args: {} # Optional


# Plot values that are collected in every time step.
# Thus, being arrays for each episode.
# Possible values are:
# - curvature
# - normalized_curvature
# - roughness
# - path_length_values
# - acceleration
# - jerk
# - velocity

#  It is possible to plot
#  - A line plot to show the course in a single episode
#    You can list multiple value to create multiple plots
single_episode_line:
  # Name of the coloumn you want to plot
  - data_key: string # Required
    # Number of values that should be skipped to reduce datapoints
    step_size: int # Optional -> Defaults to 5
    # Coloumn for differentiation
    hue: "namespace" # Optional -> Defaults to namespace
    # Index of the episode -> If none all episodes are plotted
    episode: int # Optional -> Defaults to none
    title: string
    save_name: string
    plot_args: {} # Optional
# - A Distributional plot for a single episode
#   You can list multiple value to create multiple plots
single_episode_distribution:
  - data_key: string
    episode: int
    plot_key: "swarm" | "violin" | "box" | "boxen" | "strip" # Optional -> Defaults to "swarm"
    title: string
    save_name: string
    plot_args: {} # Optional
# - A line plot showing aggregated values for all episodes.
#   Like a line plot for the max value of each episode
aggregated_distribution:
  - data_key: string
    # Function that should be used for aggregation. We offer: max, min, mean
    aggregate: "max" | "min" | "mean" | "sum"
    # Name of the dist plot you want to use. Can be strip, swarm, box, boxen, violin
    plot_key: "swarm" | "violin" | "box" | "boxen" | "strip" # Optional -> Defaults to "swarm"
    title: string
    save_name: string
    plot_args: {} # Optional
# - A distributional plot for aggregated values for all episodes.
aggregated_line:
  - data_key: string
    # Function that should be used for aggregation. We offer: max, min, mean
    aggregate: "max" | "min" | "mean" | "sum"
    title: string
    save_name: string
    plot_args: {} # Optional


## Plot values that are collected for each episode.
# Single values for each episode
# Possible values are:
# - time_diff
# - angle_over_length
# - path_length

# It is possible to plot
# - A categorical plot over all episodes to show the values in a line or bar plot
all_episodes_categorical:
  - data_key: string
    plot_key: "line" | "bar"
    title: string
    save_name: string
    plot_args: {} # Optional
# - Plot a distribution over all episodes
all_episodes_distribution:
  - data_key: string
    plot_key: "swarm" | "violin" | "box" | "boxen" | "strip" # Optional -> Defaults to "swarm"
    title: string
    save_name: string
    plot_args: {} # Optional


## Plot the path the robots took

# Plot all paths of all episodes for each robot
episode_plots_for_namespaces:
    # list of desired results that should be plotted
    desired_results: ("TIMEOUT" | "GOAL_REACHED" | "COLLISION")[]
    # Wether or not to add the obstacles from the scenario file to the plot
    should_add_obstacles: boolean # Optional -> Defaults to False
    # Wether or not to mark where collisions happened
    should_add_collisions: boolean # Optional -> Defaults to False
    title: string
    save_name: string

# Plot the best path of each robot
# Only select the paths that reached the goal and take the path that took the least amount of time
create_best_plots:
    # Wether or not to add the obstacles from the scenario file to the plot
    should_add_obstacles: boolean # Optional -> Defaults to False
    # Wether or not to mark where collisions happened
    should_add_collisions: boolean # Optional -> Defaults to False
    title: string
    save_name: string

```
