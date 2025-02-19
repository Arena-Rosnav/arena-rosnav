#!/usr/bin/env python3
import os
import random
import yaml
from PIL import Image, ImageDraw

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

# Output directory structure: map_generated/map/...

map_name = "map_" + str(random.randint(0,1000))


OUTPUT_DIR = os.path.join(os.environ['ARENA_WS_DIR'],"src","arena","simulation-setup","worlds", map_name)
os.makedirs(OUTPUT_DIR, exist_ok=True)

def generate_rooms_for_side(side, num_rooms):
    """
    Generate a list of room dictionaries for one side of the hallway.
    Rooms will have randomized widths and heights.
    The rooms’ widths are initially sampled (big rooms for first/last, small otherwise)
    and then normalized so that their total width exactly spans MAP_WIDTH.
    For top rooms, the room's bottom equals HALLWAY_TOP.
    For bottom rooms, the room's top equals HALLWAY_BOTTOM.
    A door gap (an opening) is computed along the wall touching the hallway.
    """
    rooms = []
    widths = []
    heights = []
    # Sample widths and heights (without placement)
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
    # Normalize widths to span exactly MAP_WIDTH
    norm_factor = MAP_WIDTH / total_width
    widths = [w * norm_factor for w in widths]
    
    current_x = 0.0
    for i in range(num_rooms):
        w = round(widths[i], 1)
        h = round(heights[i], 1)
        if side == "top":
            y = HALLWAY_TOP
        else:  # bottom side: room sits below the hallway
            y = HALLWAY_BOTTOM - h
        # Compute door gap on the wall that touches the hallway.
        # For top rooms, door is on the bottom wall; for bottom rooms, door is on the top wall.
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
    """
    Generate rooms for both the top and bottom sides of the hallway.
    Returns a combined list of room dictionaries.
    """
    top_rooms = generate_rooms_for_side("top", ROOMS_PER_SIDE)
    bottom_rooms = generate_rooms_for_side("bottom", ROOMS_PER_SIDE)
    return top_rooms + bottom_rooms

def room_to_zone(room):
    """
    Convert a room dict to a zone dictionary.
    The zone polygon is defined as the room's four corners in order:
    bottom-left, bottom-right, top-right, top-left.
    """
    x, y, w, h = room["x"], room["y"], room["w"], room["h"]
    polygon = [
        FlowList([x, y]),
        FlowList([x + w, y]),
        FlowList([x + w, y + h]),
        FlowList([x, y + h])
    ]
    return {"polygon": polygon}

# --- Wall Generation Functions ---
def generate_room_walls(rooms):
    """
    For each room, generate its four walls.
    The wall that touches the hallway gets a door gap.
    For top rooms, that is the bottom wall.
    For bottom rooms, that is the top wall.
    Each wall segment is wrapped in a FlowList for inline YAML formatting.
    """
    walls = []
    for room in rooms:
        x, y, w, h = room["x"], room["y"], room["w"], room["h"]
        door_start, door_end = room["door_gap"]

        if room["side"] == "top":
            # Bottom wall (touching hallway) with door gap
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
            # Other walls: right, top, left (continuous)
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
            # For bottom rooms, the wall touching the hallway is the top wall.
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
            # Other walls: right, bottom, left (continuous)
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
    """
    Generate walls along both sides of the hallway.
    For the top side of the hallway (y = HALLWAY_TOP), create a continuous wall
    that has door gaps corresponding to top rooms.
    Similarly for the bottom side of the hallway (y = HALLWAY_BOTTOM) for bottom rooms.
    """
    # Sort rooms by x-coordinate per side.
    top_rooms = sorted([r for r in rooms if r["side"] == "top"], key=lambda r: r["x"])
    bottom_rooms = sorted([r for r in rooms if r["side"] == "bottom"], key=lambda r: r["x"])

    hallway_walls = []

    # Top hallway wall at y = HALLWAY_TOP.
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

    # Bottom hallway wall at y = HALLWAY_BOTTOM.
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

# --- Utility Functions ---
def save_yaml(data, filepath):
    """Save the provided data (a dict or list) to a YAML file."""
    with open(filepath, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)

def generate_map_yaml(filepath):
    """
    Create the map.yaml file.
    The image field references the generated map image.
    """
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
    """
    Create a PNG image of the map.
    The hallway is drawn as a light gray rectangle.
    Rooms (zones) are drawn with colored outlines, and walls are drawn as black lines.
    """
    img = Image.new("RGB", (IMAGE_WIDTH, IMAGE_HEIGHT), "white")
    draw = ImageDraw.Draw(img)
    
    def to_pixel(coord):
        x, y = coord
        px = x * PIXEL_SCALE
        py = IMAGE_HEIGHT - y * PIXEL_SCALE
        return (px, py)
    
    # Draw hallway as a light-gray rectangle.
    hallway_poly = [
        (0, HALLWAY_BOTTOM),
        (MAP_WIDTH, HALLWAY_BOTTOM),
        (MAP_WIDTH, HALLWAY_TOP),
        (0, HALLWAY_TOP)
    ]
    pixel_hallway = [to_pixel(pt) for pt in hallway_poly]
    draw.polygon(pixel_hallway, fill="lightgray")
    
    # Draw rooms with random outline colors.
    for room in rooms:
        x, y, w, h = room["x"], room["y"], room["w"], room["h"]
        poly = [(x, y), (x+w, y), (x+w, y+h), (x, y+h)]
        pixel_poly = [to_pixel(pt) for pt in poly]
        color = (random.randint(0,255), random.randint(0,255), random.randint(0,255))
        draw.polygon(pixel_poly, outline=color)
    
    # Draw all walls as black lines.
    for wall in all_walls:
        start, end = wall
        draw.line([to_pixel(start), to_pixel(end)], fill="black", width=3)
    
    img.save(filepath)

# --- Main Script ---
def main():
    # Generate rooms for both sides of the hallway.
    rooms = generate_all_rooms()
    
    # Generate zones from room bounds.
    zones = [room_to_zone(room) for room in rooms]
    
    # Generate room walls (with door gaps on the side that touches the hallway).
    room_walls = generate_room_walls(rooms)
    # Generate hallway walls for both the top and bottom edges.
    hallway_walls = generate_hallway_walls(rooms)
    # Combine all wall segments.
    all_walls = room_walls + hallway_walls
    
    # Write zones.yaml.
    zones_yaml_path = os.path.join(OUTPUT_DIR, "zones.yaml")
    save_yaml(zones, zones_yaml_path)
    print("Saved zones.yaml to", zones_yaml_path)
    
    # Write walls.yaml (wrapped in a dictionary with key "walls").
    walls_yaml_path = os.path.join(OUTPUT_DIR, "walls.yaml")
    save_yaml({"walls": all_walls}, walls_yaml_path)
    print("Saved walls.yaml to", walls_yaml_path)
    
    # Write map.yaml.
    map_yaml_path = os.path.join(OUTPUT_DIR, "map.yaml")
    generate_map_yaml(map_yaml_path)
    print("Saved map.yaml to", map_yaml_path)
    
    # Generate and save the map image with the random map name.
    map_png_path = os.path.join(OUTPUT_DIR, map_name + ".png")
    draw_map_image(rooms, all_walls, map_png_path)
    print("Saved", map_png_path)

if __name__ == "__main__":
    main()
