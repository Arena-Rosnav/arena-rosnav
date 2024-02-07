#!/usr/bin/env python

import rospy
import rospkg
import matplotlib.pyplot as plt
import os 
import threading
import subprocess

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovariance
from nav_msgs.msg import Path
from std_srvs.srv import Empty,EmptyRequest


class GridMapGenerator:
    def __init__(self):

        rospy.init_node("grid_map_generator", anonymous=True)
        
        world_name = rospy.get_param('~world_name', None)

        if world_name is None or world_name == '':
            rospy.logerr("No world_name argument provided! Please provide the world_name argument.")
            rospy.signal_shutdown("No world_name argument provided.")
            return
        
        # Create folder inside /arena_simulation_setup/maps 
        self.timer0_  = threading.Thread(target=self.create_folder, args=(world_name,))
        self.timer0_ .start()

        # Wait until Gazebo is up then converting 
        self.wait_for_gazebo_service()

        # Call the service rosservice call /gazebo_2Dmap_plugin/generate_map
        self.timer1_  = threading.Thread(target=self.grid_map_generator, args=())
        self.timer1_ .start()

        # Save the map.pgm and the map.yaml file in maps folder 
        self.timer3_  = threading.Thread(target=self.save_map, args=(world_name,))
        self.timer3_ .start()

        # Waits for self.timer3_ to finish before proceeding to start self.timer4_
        self.timer3_.join()

        # Write map.pgm to the first line of the map.yaml file
        self.timer4_  = threading.Thread(target=self.write_data, args=(world_name,))
        self.timer4_ .start()

        # Waits for self.timer4_ to finish before proceeding to start self.timer5_
        self.timer4_.join()

        # Write map.pgm to the first line of the map.yaml file
        self.timer5_  = threading.Thread(target=self.create_file, args=(world_name,))
        self.timer5_ .start()

    def create_folder(self, world_name):

        folder_name = world_name  
        rospack = rospkg.RosPack()
        arena_pkg_path = rospack.get_path("arena_simulation_setup")

        maps_dir = os.path.join(arena_pkg_path, "worlds")
        folder_path = os.path.join(maps_dir, folder_name, "map") 

        try:
        # Create the folder/directory
           if not os.path.exists(folder_path):
               os.makedirs(folder_path)
               print(f"Folder '{world_name}' created successfully.")
           else:
               print(f"Folder '{world_name}' already exist.")
        except OSError as e:
            print(f"Error creating folder '{world_name}': {e}")

    def wait_for_gazebo_service(self):
        service_name = "/gazebo/spawn_urdf_model"
        rospy.loginfo(f"Waiting for Gazebo service: {service_name}")
        rospy.wait_for_service(service_name)
        rospy.loginfo("Gazebo service is available.")

    def grid_map_generator(self):

        rospy.wait_for_service('/gazebo_2Dmap_plugin/generate_map')
        try:
            grid_map_generator = rospy.ServiceProxy('/gazebo_2Dmap_plugin/generate_map', Empty)
            request = EmptyRequest()
            response = grid_map_generator(request)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def save_map(self, world_name):
        # Run the map_saver command using subprocess
        folder_name = world_name 
        rospack = rospkg.RosPack()
        arena_pkg_path = rospack.get_path("arena_simulation_setup")
        maps_dir = os.path.join(arena_pkg_path, "worlds")
        folder_path = os.path.join(maps_dir, folder_name, "map") 
        map_saver_command = ['rosrun', 'map_server', 'map_saver', '-f', folder_path, '/map:=/map2d']
        subprocess.run(map_saver_command)
        rospy.loginfo("Map saved successfully.")

    def write_data(self, world_name):

        folder_name = world_name 
        rospack = rospkg.RosPack()
        arena_pkg_path = rospack.get_path("arena_simulation_setup")
        maps_dir = os.path.join(arena_pkg_path, "worlds")
        folder_path = os.path.join(maps_dir, folder_name, "map")      
        file_path = os.path.join(folder_path, "map.yaml")    
   
        # Read existing content
        with open(file_path, 'r') as file:
         lines = file.readlines()

        # Modify the first line
        if lines:
         lines[0] = "image: map.pgm\n"

        # Write back the modified content
        with open(file_path, 'w') as file:
         file.writelines(lines)

    def create_file(self, world_name):

        file_content = """properties:
  velocity_iterations: 10
  position_iterations: 10
layers:
- name: static
  map: map.yaml
  color: [0, 1, 0, 1]
"""

        folder_name = world_name 
        file_name = "map.world.yaml"
        rospack = rospkg.RosPack()
        arena_pkg_path = rospack.get_path("arena_simulation_setup")
        maps_dir = os.path.join(arena_pkg_path, "worlds")
        folder_path = os.path.join(maps_dir, folder_name)     
        file_path = os.path.join(folder_path, "map", file_name) 

        try:
         with open(file_path, 'w') as file:
            file.write(file_content)
         print(f"File '{file_name}' created successfully.")
        except OSError as e:
         print(f"Error creating file '{file_name}': {e}")

def main():

    try:
       
       while not rospy.is_shutdown(): 
          GridMapGenerator()
          rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated!")

if __name__ == '__main__':  
    main()