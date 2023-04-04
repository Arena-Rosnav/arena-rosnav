from pathlib import Path

MAP_FOLDER_NAME = "dynamic_map"
EMPTY_MAP_YAML = {
    "image": "map.png",
    "resolution": 0.25,
    "origin": [0.0, 0.0, 0.0],  # [-x,-y,0.0]
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196,
}

DYNAMIC_MAP_YAML = {
    "image": f"{MAP_FOLDER_NAME}.png",
    "resolution": 0.25,
    "origin": [0.0, 0.0, 0.0],  # [-x,-y,0.0]
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196,
}
ROSNAV_MAP_FOLDER = (
    Path(f"{__file__}").parent.parent.parent.parent.parent
    / "utils"
    / "arena-simulation-setup"
    / "maps"
)
