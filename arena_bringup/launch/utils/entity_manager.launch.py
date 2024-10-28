import os
import launch
import launch_ros.actions

def generate_launch_description():
   # Current path to the workspace
   workspace_dir = os.path.join(os.getenv('HOME'), 'arena4_ws/src/arena/arena-rosnav/arena_bringup/launch/utils')

   ld = launch.LaunchDescription([
       # Declare arguments for entity manager and world file
       launch.actions.DeclareLaunchArgument(
           name='entity_manager',
           default_value='hunavsim'  # Set hunavsim as default
       ),
       launch.actions.DeclareLaunchArgument(
           name='world_file',
           default_value=''
       ),
       launch.actions.DeclareLaunchArgument(
           name='scene_file',
           default_value=launch.substitutions.LaunchConfiguration('world_file')
       ),
       
       
       launch.actions.IncludeLaunchDescription(
           launch.launch_description_sources.PythonLaunchDescriptionSource(
               os.path.join(workspace_dir, 'hunavsim.launch.py')
           ),
           launch_arguments={
               'use_sim_time': 'true',
               'world_file': launch.substitutions.LaunchConfiguration('world_file')
           }.items()
       )
   ])
   return ld

if __name__ == '__main__':
   generate_launch_description()
