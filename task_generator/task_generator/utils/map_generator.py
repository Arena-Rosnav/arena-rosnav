#!/usr/bin/env python3
import os
import random
import yaml
from PIL import Image, ImageDraw
import rclpy
from rclpy.node import Node

# --- Define a custom list type for inline (flow) YAML formatting ---
class FlowList(list):
    pass

def flow_list_representer(dumper, data):
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)

yaml.add_representer(FlowList, flow_list_representer)

# --- Parameters ---
MAP_WIDTH = 60.0
MAP_HEIGHT = 60.0
RESOLUTION = 0.05          # Same as in map.yaml
PIXEL_SCALE = int(1 / RESOLUTION)  # e.g., 20 pixels per unit
IMAGE_WIDTH = int(MAP_WIDTH * PIXEL_SCALE)
IMAGE_HEIGHT = int(MAP_HEIGHT * PIXEL_SCALE)

# Hallway parameters (a horizontal band)
HALLWAY_BOTTOM = 28.0
HALLWAY_TOP = 32.0   # Hallway height is 4 units

# Room parameters (for each side)
ROOMS_PER_SIDE = 5

# For "big" rooms (first and last)
BIG_MIN_WIDTH = 15.0
BIG_MAX_WIDTH = 25.0
BIG_MIN_HEIGHT = 15.0
BIG_MAX_HEIGHT = 25.0

# For "small" rooms (intermediate ones)
SMALL_MIN_WIDTH = 5.0
SMALL_MAX_WIDTH = 15.0
SMALL_MIN_HEIGHT = 8.0
SMALL_MAX_HEIGHT = 12.0

DOOR_WIDTH = 2.0  # door gap along the wall where room meets hallway

# --- Map Generation Functions ---
def generate_rooms_for_side(side, num_rooms):
    rooms = []
    widths = []
    heights = []
    for i in range(num_rooms):
        if i == 0 or i == num_rooms - 1:
            w = random.uniform(BIG_MIN_WIDTH, BIG_MAX_WIDTH)
            h = random.uniform(BIG_MIN_HEIGHT, BIG_MAX_HEIGHT)
        else:
            w = random.uniform(SMALL_MIN_WIDTH, SMALL_MAX_WIDTH)
            h = random.uniform(SMALL_MIN_HEIGHT, SMALL_MAX_HEIGHT)
        widths.append(w)
        heights.append(h)
    total_width = sum(widths)
    norm_factor = MAP_WIDTH / total_width
    widths = [w * norm_factor for w in widths]
    
    current_x = 0.0
    for i in range(num_rooms):
        w = round(widths[i], 1)
        h = round(heights[i], 1)
        if side == "top":
            y = HALLWAY_TOP
        else:
            y = HALLWAY_BOTTOM - h
        if w > DOOR_WIDTH:
            door_start = random.uniform(current_x + DOOR_WIDTH, current_x + w - DOOR_WIDTH)
            door_end = door_start + DOOR_WIDTH
        else:
            door_start, door_end = current_x, current_x + w
        room = {
            "x": round(current_x, 1),
            "y": round(y, 1),
            "w": w,
            "h": h,
            "side": side,
            "door_gap": (round(door_start, 1), round(door_end, 1))
        }
        rooms.append(room)
        current_x += w
    return rooms

def generate_all_rooms():
    top_rooms = generate_rooms_for_side("top", ROOMS_PER_SIDE)
    bottom_rooms = generate_rooms_for_side("bottom", ROOMS_PER_SIDE)
    return top_rooms + bottom_rooms

def room_to_zone(room):
    x, y, w, h = room["x"], room["y"], room["w"], room["h"]
    polygon = [
        FlowList([x, y]),
        FlowList([x + w, y]),
        FlowList([x + w, y + h]),
        FlowList([x, y + h])
    ]
    return {"polygon": polygon}

def generate_room_walls(rooms):
    walls = []
    for room in rooms:
        x, y, w, h = room["x"], room["y"], room["w"], room["h"]
        door_start, door_end = room["door_gap"]

        if room["side"] == "top":
            if door_start - x > 0.1:
                walls.append(FlowList([
                    FlowList([x, y]),
                    FlowList([door_start, y])
                ]))
            if (x + w) - door_end > 0.1:
                walls.append(FlowList([
                    FlowList([door_end, y]),
                    FlowList([x + w, y])
                ]))
            walls.append(FlowList([
                FlowList([x + w, y]),
                FlowList([x + w, y + h])
            ]))
            walls.append(FlowList([
                FlowList([x + w, y + h]),
                FlowList([x, y + h])
            ]))
            walls.append(FlowList([
                FlowList([x, y + h]),
                FlowList([x, y])
            ]))
        else:
            if (x + w) - door_end > 0.1:
                walls.append(FlowList([
                    FlowList([x + w, y + h]),
                    FlowList([door_end, y + h])
                ]))
            if door_start - x > 0.1:
                walls.append(FlowList([
                    FlowList([door_start, y + h]),
                    FlowList([x, y + h])
                ]))
            walls.append(FlowList([
                FlowList([x + w, y + h]),
                FlowList([x + w, y])
            ]))
            walls.append(FlowList([
                FlowList([x + w, y]),
                FlowList([x, y])
            ]))
            walls.append(FlowList([
                FlowList([x, y]),
                FlowList([x, y + h])
            ]))
    return walls

def generate_hallway_walls(rooms):
    top_rooms = sorted([r for r in rooms if r["side"] == "top"], key=lambda r: r["x"])
    bottom_rooms = sorted([r for r in rooms if r["side"] == "bottom"], key=lambda r: r["x"])

    hallway_walls = []

    segments = []
    current_x = 0.0
    for room in top_rooms:
        door_start, door_end = room["door_gap"]
        if door_start > current_x + 0.1:
            segments.append((current_x, door_start))
        current_x = max(current_x, door_end)
    if current_x < MAP_WIDTH - 0.1:
        segments.append((current_x, MAP_WIDTH))
    for seg in segments:
        x1, x2 = seg
        hallway_walls.append(FlowList([
            FlowList([round(x1, 1), HALLWAY_TOP]),
            FlowList([round(x2, 1), HALLWAY_TOP])
        ]))

    segments = []
    current_x = 0.0
    for room in bottom_rooms:
        door_start, door_end = room["door_gap"]
        if door_start > current_x + 0.1:
            segments.append((current_x, door_start))
        current_x = max(current_x, door_end)
    if current_x < MAP_WIDTH - 0.1:
        segments.append((current_x, MAP_WIDTH))
    for seg in segments:
        x1, x2 = seg
        hallway_walls.append(FlowList([
            FlowList([round(x1, 1), HALLWAY_BOTTOM]),
            FlowList([round(x2, 1), HALLWAY_BOTTOM])
        ]))
    return hallway_walls

def save_yaml(data, filepath):
    with open(filepath, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)

def generate_map_yaml(filepath, map_name):
    map_yaml = {
        "free_thresh": 0.196,
        "image": map_name + ".png",
        "negate": 0,
        "occupied_thresh": 0.65,
        "origin": [0, 0, 0],
        "resolution": RESOLUTION,
        "type": "indoor"
    }
    save_yaml(map_yaml, filepath)

def draw_map_image(rooms, all_walls, filepath):
    img = Image.new("RGB", (IMAGE_WIDTH, IMAGE_HEIGHT), "white")
    draw = ImageDraw.Draw(img)
    
    def to_pixel(coord):
        x, y = coord
        px = x * PIXEL_SCALE
        py = IMAGE_HEIGHT - y * PIXEL_SCALE
        return (px, py)
    
    hallway_poly = [
        (0, HALLWAY_BOTTOM),
        (MAP_WIDTH, HALLWAY_BOTTOM),
        (MAP_WIDTH, HALLWAY_TOP),
        (0, HALLWAY_TOP)
    ]
    pixel_hallway = [to_pixel(pt) for pt in hallway_poly]
    draw.polygon(pixel_hallway, fill="lightgray")
    
    for room in rooms:
        x, y, w, h = room["x"], room["y"], room["w"], room["h"]
        poly = [(x, y), (x+w, y), (x+w, y+h), (x, y+h)]
        pixel_poly = [to_pixel(pt) for pt in poly]
        color = (random.randint(0,255), random.randint(0,255), random.randint(0,255))
        draw.polygon(pixel_poly, outline=color)
    
    for wall in all_walls:
        start, end = wall
        draw.line([to_pixel(start), to_pixel(end)], fill="black", width=3)
    
    img.save(filepath)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('map_generator')
    
    # Declare ROS2 parameters.
    arena_ws_dir = os.environ['ARENA_WS_DIR']
    default_map_name = "map_" + str(random.randint(0, 1000))
    node.declare_parameter('map_name', default_map_name)
    map_name = node.get_parameter('map_name').value
    
    # Create the output directory based on the provided map_name.
    OUTPUT_DIR = os.path.join(arena_ws_dir, "src", "arena", "simulation-setup", "worlds", map_name)
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # Generate rooms, zones, and walls.
    rooms = generate_all_rooms()
    zones = [room_to_zone(room) for room in rooms]
    room_walls = generate_room_walls(rooms)
    hallway_walls = generate_hallway_walls(rooms)
    all_walls = room_walls + hallway_walls
    
    # Save YAML files.
    zones_yaml_path = os.path.join(OUTPUT_DIR, "zones.yaml")
    save_yaml(zones, zones_yaml_path)
    node.get_logger().info("Saved zones.yaml to " + zones_yaml_path)
    
    walls_yaml_path = os.path.join(OUTPUT_DIR, "walls.yaml")
    save_yaml({"walls": all_walls}, walls_yaml_path)
    node.get_logger().info("Saved walls.yaml to " + walls_yaml_path)
    
    map_yaml_path = os.path.join(OUTPUT_DIR, "map.yaml")
    generate_map_yaml(map_yaml_path, map_name)
    node.get_logger().info("Saved map.yaml to " + map_yaml_path)
    
    map_png_path = os.path.join(OUTPUT_DIR, map_name + ".png")
    draw_map_image(rooms, all_walls, map_png_path)
    node.get_logger().info("Saved map image to " + map_png_path)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
