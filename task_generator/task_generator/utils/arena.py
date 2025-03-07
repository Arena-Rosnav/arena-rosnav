

import os
import numpy as np

from task_generator.constants import Constants

import nav_msgs.msg as nav_msgs

import ament_index_python.packages

import re
import tempfile
import xml.etree.ElementTree as ET


_ARENA_WS_DIR = os.path.realpath(os.path.join(ament_index_python.packages.get_package_share_directory('task_generator'), '..', '..', '..', '..'))

_SS_PATH: str = ament_index_python.packages.get_package_share_directory('arena_simulation_setup')


def get_arena_ws() -> str:
    """
    Get path to ARENA_WS_DIR
    """
    return _ARENA_WS_DIR


def get_simulation_setup_path() -> str:
    """
    Get path to arena_simulation_setup package (src).
    """
    return _SS_PATH


def get_arena_type() -> Constants.ArenaType:
    """
    Get arena type.
    """
    return Constants.ArenaType(
        os.getenv("ARENA_TYPE", Constants.ArenaType.DEPLOYMENT.value).lower())


def generate_map_inner_border(free_space_indices,
                              map_: nav_msgs.OccupancyGrid):
    """generate map border (four vertices of the map)

    Returns:
        vertex_coordinate_x_y(np.ndarray with shape 4 x 2):
    """
    n_freespace_cells = len(free_space_indices[0])
    border_vertex = np.array([]).reshape(0, 2)
    border_vertices = np.array([]).reshape(0, 2)
    for idx in [0, n_freespace_cells - 4]:
        y_in_cells, x_in_cells = free_space_indices[0][idx], free_space_indices[1][idx]
        y_in_meters = y_in_cells * map_.info.resolution + map_.info.origin.position.y
        x_in_meters = x_in_cells * map_.info.resolution + map_.info.origin.position.x
        border_vertex = np.vstack(
            [border_vertex, [x_in_meters, y_in_meters]])
    border_vertices = np.vstack(
        [border_vertices, [border_vertex[0, 0], border_vertex[0, 1]]])
    border_vertices = np.vstack(
        [border_vertices, [border_vertex[0, 0], border_vertex[1, 1]]])
    border_vertices = np.vstack(
        [border_vertices, [border_vertex[1, 0], border_vertex[1, 1]]])
    border_vertices = np.vstack(
        [border_vertices, [border_vertex[1, 0], border_vertex[0, 1]]])
    # print('border',border_vertices)
    return border_vertices


def update_freespace_indices_maze(map_: nav_msgs.OccupancyGrid):
    """update the indices(represented in a tuple) of the freespace based on the map and the static polygons
    ostacles manuelly added
    param map_ : original occupacy grid
    param vertlist: vertex of the polygons

    Returns:
        indices_y_x(tuple): indices of the non-occupied cells, the first element is the y-axis indices,
        the second element is the x-axis indices.
    """
    width_in_cell, height_in_cell = map_.info.width, map_.info.height
    map_2d = np.reshape(map_.data, (height_in_cell, width_in_cell))
    # height range and width range
    wall_occupancy = np.array([[1.25, 12.65, 10.6, 10.8],
                               [-4.45, 18.35, 16.3, 16.5],
                               [-4.45, 18.35, 4.9, 5.1],
                               [12.55, 12.75, -0.7, 22.1],
                               [1.15, 1.35, -0.7, 22.1],
                               [6.85, 7.05, 5.0, 16.4]])
    size = wall_occupancy.shape[0]
    for ranges in wall_occupancy:
        height_low = int(ranges[0] / map_.info.resolution)
        height_high = int(ranges[1] / map_.info.resolution)
        width_low = int(ranges[2] / map_.info.resolution)
        width_high = int(ranges[3] / map_.info.resolution)
        height_grid = height_high - height_low
        width_grid = width_high - width_low
        for i in range(height_grid):
            y = height_low + i
            for j in range(width_grid):
                x = width_low + j
                map_2d[y, x] = 100
    free_space_indices_new = np.where(map_2d == 0)
    return free_space_indices_new

def process_dae(dae_file, package_dir):
    """
    Load a .dae file, update its <init_from> elements by replacing any leading
    '../' with the package_dir, then write to a temporary file and return its path.
    """
    import collada
    file = collada.Collada(dae_file)
    tree = file.xmlnode
    root = tree.getroot()
    for init_elem in root.iterfind('.//init_from'):
        print(init_elem)
        if init_elem.text:
            text = init_elem.text.strip()
            if text.startswith("../"):
                # Remove all leading "../" segments
                rel_path = text
                while rel_path.startswith("../"):
                    rel_path = rel_path[3:]
                # Create a new absolute path using the package directory
                new_text = os.path.join(package_dir, rel_path)
                init_elem.text = new_text

    with tempfile.NamedTemporaryFile(mode="wb", suffix=".dae", delete=False) as tmp_file:
        # Write the XML tree to the temporary file.
        tree.write(tmp_file, pretty_print=True, xml_declaration=True, encoding="UTF-8")
        temp_filename = tmp_file.name
    print(temp_filename)
    return temp_filename


    # Write the updated .dae file to a temporary file
    

def process_obj(obj_file, package_dir):
    """
    Read an .obj file as text and update any .png file references.
    For any found relative .png path (e.g. starting with "../"), remove the
    relative segments and prepend the package_dir. The modified file is saved
    to a temporary file whose path is returned.
    """
    try:
        with open(obj_file, 'r', encoding='utf-8') as f:
            content = f.read()
    except Exception as e:
        print(f"Error reading {obj_file}: {e}")
        return obj_file  # fallback: return original file if error occurs

    # Regex to match .png filenames (non-space characters ending in .png)
    png_pattern = re.compile(r'(?P<path>\S+\.png)')
    mtl_patter = re.compile(r'(?P<path>\S+\.mtl)')
    def replace_png(match):
        path = match.group("path")
        # If already absolute, do nothing.
        if os.path.isabs(path):
            return path
        # Remove any leading '../' segments
        while path.startswith("../"):
            path = path[3:]
        # Return the absolute path by joining with the package directory
        return os.path.join(package_dir, path)

    new_content = png_pattern.sub(replace_png, content)

    # Write the updated .obj file to a temporary file
    with tempfile.NamedTemporaryFile(delete=False, suffix='.obj', mode='w', encoding='utf-8') as temp_file:
        temp_file.write(new_content)
        temp_filename = temp_file.name
    print(temp_filename)
    return temp_filename
