class Config:
    MAP = {
        "Alpha": 0.6,
        "Class": "rviz/Map",
        "Color Scheme": "map",
        "Draw Behind": "true",
        "Enabled": "true",
        "Name": "Map",
        "Topic": "/map",
        "Unreliable": "false",
        "Use Timestamp": "false",
        "Value": "true",
    }
    TF = {
        "Class": "rviz/TF",
        "Enabled": True,
        "Frame Timeout": 15,
        "Frames": {
            "All Enabled": True
        },
        "Marker Alpha": 1,
        "Marker Scale": 1,
        "Name": "Transform",
        "Show Arrows": False,
        "Show Axes": True,
        "Show Names": True,
        "Update Interval": 0,
        "Value": True,
    }

    TRACKED_PERSONS = {
        "Alpha": 1,
        "Class": "spencer_tracking_rviz_plugin/TrackedPersons",
        "Color": "130; 130; 130",
        "Color map offset": 0,
        "Color transform": "SRL Tracking Colors",
        "Delete after no. cycles": 100,
        "Enabled": True,
        "Excluded person IDs": "",
        "Font color": "255; 255; 255",
        "Font color style": "Same color",
        "Font scale": 2,
        "History as line": {
            "Line width": 0.05000000074505806,
            "Value": False
        },
        "History size": 100,
        "Included person IDs": "",
        "Min. history point distance": 0.4000000059604645,
        "Missed alpha": 1,
        "Name": "TrackedPersons",
        "Occlusion alpha": 0.30000001192092896,
        "Queue Size": 10,
        "Render covariances": {
            "Line width": 0.10000000149011612,
            "Value": True
        },
        "Render detection IDs": False,
        "Render history": False,
        "Render person visual": True,
        "Render track IDs": False,
        "Render track state": False,
        "Render velocities": True,
        "Show DELETED tracks": False,
        "Show MATCHED tracks": True,
        "Show MISSED tracks": True,
        "Show OCCLUDED tracks": True,
        "Style": {
            "Line width": 0.05,
            "Scaling factor": 1,
            "Value": "Person meshes"
        },
        "Topic": "/pedsim_visualizer/tracked_persons",
        "Unreliable": False,
        "Value": True,
        "Z offset": {
            "Use Z position from message": False,
            "Value": 0
        }
    }

    TRACKED_GROUPS = {
        "Alpha": 1,
        "Class": "spencer_tracking_rviz_plugin/TrackedGroups",
        "Color": "130; 130; 130",
        "Color map offset": 0,
        "Color transform": "SRL Tracking Colors",
        "Connect group members": True,
        "Enabled": False,
        "Excluded group IDs": "",
        "Excluded person IDs": "",
        "Font color": "255; 255; 255",
        "Font color style": "Same color",
        "Font scale": 2,
        "Global history size": 1000,
        "Group ID Z offset": 2,
        "Included group IDs": "",
        "Included person IDs": "",
        "Name": "TrackedGroups",
        "Occlusion alpha": 0.5,
        "Queue Size": 10,
        "Render group IDs": {
            "Hide IDs of single-person groups": False,
            "Value": True
        },
        "Render history": False,
        "Single-person groups in constant color": True,
        "Style": {
            "Line width": 0.05,
            "Scaling factor": 1,
            "Value": "Cylinders"
        },
        "Topic": "/pedsim_visualizer/tracked_groups",
        "Tracked persons topic": "",
        "Unreliable": False,
        "Value": True,
        "Z offset": {
            "Use Z position from message": False,
            "Value": 0
        }
    }

    PEDSIM_WALLS = {
        "Class": "rviz/Marker",
        "Enabled": True,
        "Marker Topic": "/pedsim_visualizer/walls",
        "Name": "PedsimWalls",
        "Namespaces": {
            "walls": True
        },
        "Queue Size": 100,
        "Value": True
    }

    PEDSIM_WAYPOINTS = {
        "Class": "rviz/MarkerArray",
        "Enabled": False,
        "Marker Topic": "/pedsim_visualizer/waypoints",
        "Name": "PedsimWaypoints",
        "Namespaces": {},
        "Queue Size": 100,
        "Value": False
    }

    def create_model_display(robot_name, topic, color):
        return {
            "Class": "rviz/MarkerArray",
            "Enabled": True,
            "Marker Topic": topic,
            "Name":  f"{robot_name} MarkerArray",
            "Namespaces": {
                "": True
            },
            "Queue Size": 100,
            "Value": True
        }

    def create_pose_display(robot_name, topic, color):
        return {
            "Alpha": 1,
            "Axes Length": 1,
            "Axes Radius": 0.1,
            "Class": "rviz/Pose",
            "Color": color,
            "Enabled": True,
            "Head Length": 0.1,
            "Head Radius": 0.15,
            "Name": f"{robot_name} {'Subgoal' if 'subgoal' in topic else 'Goal'}",
            "Queue Size": 10,
            "Shaft Length": 0.5,
            "Shaft Radius": 0.03,
            "Shape": "Arrow",
            "Topic": topic,
            "Unreliable": False,
            "Value": False
        }

    def create_global_map_display(robot_name, topic, _):
        return Config._create_map_display(robot_name, topic, 0.7, "Global Costmap")

    def create_local_map_display(robot_name, topic, _):
        return Config._create_map_display(robot_name, topic, 0.3, "Local Costmap")

    def _create_map_display(robot_name, topic, alpha, name):
        return {
            "Alpha": alpha,
            "Class": "rviz/Map",
            "Color Scheme": "map",
            "Draw Behind": False,
            "Enabled": False,
            "Name": f"{robot_name} {name}",
            "Topic": topic,
            "Unreliable": False,
            "Use Timestamp": False,
            "Value": True
        }

    def create_path_display(robot_name, topic, color):
        return {
            "Alpha": 1,
            "Buffer Length": 1,
            "Class": "rviz/Path",
            "Color": color,
            "Enabled": True,
            "Head Diameter": 0.3,
            "Head Length": 0.2,
            "Length": 0.3,
            "Line Style": "Lines",
            "Line Width": 0.03,
            "Name": f"{robot_name} Global Plan",
            "Offset": {
                "X": 0,
                "Y": 0,
                "Z": 0,
            },
            "Pose Color": "255; 85; 255",
            "Pose Style": None,
            "Queue Size": 10,
            "Radius": 0.03,
            "Shaft Diameter": 0.1,
            "Shaft Length": 0.1,
            "Topic": topic,
            "Unreliable": False,
            "Value": False,
        }

    def create_laser_scan_display(robot_name, topic, color):
        return {
            "Alpha": 1,
            "Autocompute Intensity Bounds": True,
            "Autocompute Value Bounds": {
                "Max Value": 0.3,
                "Min Value": 0.3,
                "Value": True
            },
            "Axis": "Z",
            "Channel Name": "intensity",
            "Class": "rviz/LaserScan",
            "Color": color,
            "Color Transformer": "FlatColor",
            "Decay Time": 0,
            "Enabled": False,
            "Invert Rainbow": False,
            "Max Color": "239; 41; 41",
            "Min Color": "164; 0; 0",
            "Name": f"{robot_name} LaserScan",
            "Position Transformer": "XYZ",
            "Queue Size": 10,
            "Selectable": True,
            "Size (Pixels)": 8,
            "Size (m)": 0.001,
            "Style": "Points",
            "Topic": topic,
            "Unreliable": False,
            "Use Fixed Frame": True,
            "Use rainbow": True,
            "Value": True
        }
