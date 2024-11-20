import os
import subprocess
from ros_introspection.util import get_packages

# Pfad zu Ihrem Workspace
workspace_path = '/home/jonas/ros2-migration_all/ros2-migration/arena/arena-rosnav'

# Pakete identifizieren
packages = get_packages(workspace_path)

# Ausgabe der identifizierten Pakete
print("Gefundene ROS1-Packages:")
for package in packages:
    if(package.name == "cohan_msgs"):
        print(package.name)

# Konvertierung der ROS1-Packages zu ROS2
print("\nKonvertierung der ROS1-Packages zu ROS2:")
for package in packages:
    package_path = os.path.join(workspace_path, package.name)
    print(f"Konvertiere {package.name}...")
    try:
        subprocess.run(['rosrun', 'magical_ros2_conversion_tool', 'ros2_conversion', package_path], check=True)
        print(f"{package.name} erfolgreich konvertiert.")
    except subprocess.CalledProcessError as e:
        print(f"Fehler bei der Konvertierung von {package.name}: {e}")

