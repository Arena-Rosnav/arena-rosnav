map_res = 0.25

MAP_FOLDER_NAME = "dynamic_map"
EMPTY_MAP_YAML = {
    "image": "empty_map.png",
    "resolution": map_res,
    "origin": [0.0, 0.0, 0.0],  # [-x,-y,0.0]
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196,
}

DYNAMIC_MAP_YAML = {
    "image": f"{MAP_FOLDER_NAME}.png",
    "resolution": map_res,
    "origin": [0.0, 0.0, 0.0],  # [-x,-y,0.0]
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196,
}
