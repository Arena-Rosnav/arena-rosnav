import base64
import io
import itertools

import json
import typing
import cv2
import numpy as np
import rospy

import requests
import shapely
import shapely.affinity
import drawsvg

from map_generator.base_map_gen import BaseMapGenerator
from map_generator.factory import MapGeneratorFactory
from map_generator.constants import (
    MapGenerators,
    MAP_GENERATOR_NS,
)

import sys

def affine_matrix_to_shapely(mat):
    mat = mat/mat[2,2]
    return [mat[0,0], mat[0,1], mat[1,0], mat[1,1], mat[0,2], mat[1,2]]

class Floorplan:

    _CAT_LABELS = {
        1:  'Living Room',
        2:  'Kitchen',
        3:  'Bedroom',
        4:  'Bathroom',
        5:  'Balcony',
        6:  'Entrance',
        7:  'Dining Room',
        8:  'Study Room',
        10: 'Storage',
        11: 'Front Door',
        13: 'Unknown',
        12: 'Interior Door',
    }

    _CAT_COLORS = {
        1: '#EE4D4D',
        2: '#C67C7B',
        3: '#FFD274',
        4: '#BEBEBE',
        5: '#BFE3E8',
        6: '#7BA779',
        7: '#E87A90',
        8: '#FF8C69',
        10: '#1F849B',
        11: '#727171',
        12: '#D3A2C7',
        13: '#785A67',
    }

    def compute_gridmap(self, gridmap):
        self._tf_floorplan_pixel = np.eye(3)

         # orient both horizontal
        if (gridmap_flipped := gridmap.shape[0] > gridmap.shape[1]):
            gridmap = gridmap.T

        if (floorplan_flipped := self._resolution[0] > self._resolution[1]):
            self._resolution = self._resolution[::-1]

        if floorplan_flipped != gridmap_flipped:
            self._tf_floorplan_pixel = np.array([
                [0,1,0],
                [1,0,0],
                [0,0,1]
            ]) @ self._tf_floorplan_pixel

        scale_to_fit = min(
            gridmap.shape[0] / self._resolution[0],
            gridmap.shape[1] / self._resolution[1]
        )

        self._tf_floorplan_pixel[2,2] /= scale_to_fit

        self._tf_floorplan_pixel = np.array([
            [-1, 0, gridmap.shape[1]], # tf2 is x-left
            [0, 1, 0],
            [0, 0, 1]
        ]) @ self._tf_floorplan_pixel

        gridmap[:,:] = 1

        for room in shapely.affinity.affine_transform(self._rooms, affine_matrix_to_shapely(self._tf_floorplan_pixel)).geoms:
            cv2.fillConvexPoly(
                gridmap,
                np.array(room.boundary.coords).astype(np.int32),
                (0,)
            )

        for room in shapely.affinity.affine_transform(self._rooms, affine_matrix_to_shapely(self._tf_floorplan_pixel)).geoms:
            cv2.polylines(
                gridmap,
                [np.array(room.boundary.coords).astype(np.int32).reshape((-1,1,2))],
                False,
                (1,),
                1
            )

        for door in shapely.affinity.affine_transform(self._doors, affine_matrix_to_shapely(self._tf_floorplan_pixel)).geoms:
            cv2.fillConvexPoly(
                gridmap,
                np.array(door.buffer(2).boundary.coords).astype(np.int32),
                (0,)
            )

        if gridmap_flipped:
            gridmap = gridmap.T

        if floorplan_flipped != gridmap_flipped:
            self._tf_floorplan_pixel = np.array([
                [0,1,0],
                [1,0,0],
                [0,0,1]
            ]) @ self._tf_floorplan_pixel

        return gridmap

    def _compute_zones(self, map_resolution):
        # pixel to world coordinates
        self._tf_pixel_world = np.eye(3)

        self._tf_pixel_world[2,2] /= map_resolution

        zones = []
        for polygon, category in zip(
            shapely.affinity.affine_transform(self._rooms, affine_matrix_to_shapely(self._tf_pixel_world @ self._tf_floorplan_pixel)).geoms,
            self._room_categories
        ):
            zones.append({
                'label': self._CAT_LABELS.get(category, 'UNKNOWN'),
                'category': [self._CAT_LABELS.get(category, 'UNKNOWN').lower()], #todo
                'polygon': list(polygon.boundary.coords)[:-1]
            })

        return zones
    
    def _visualize_zones(self):
        drawing = drawsvg.Drawing(self.gridmap.shape[1], self.gridmap.shape[0])

        tf_pixel_to_svg = np.array([
            [-1,0,self.gridmap.shape[1]],
            [0,-1,self.gridmap.shape[0]],
            [0,0,1]
        ])

        background = base64.b64encode(
            np.array(
                cv2.imencode('.png', np.flip(1 - self.gridmap, axis=(0,1)) * 255)[1]
            ).tobytes()
        ).decode()

        drawing.append(
            drawsvg.Image(
                0,
                0,
                self.gridmap.shape[1],
                self.gridmap.shape[0],
                f'data:image/png;base64,{background}'
            )
        )

        svg_coords = shapely.affinity.affine_transform(self._rooms, affine_matrix_to_shapely(tf_pixel_to_svg @ self._tf_floorplan_pixel))

        for polygon, category in zip(
            svg_coords.geoms,
            self._room_categories
        ):
            drawing.append(
                drawsvg.Lines(
                    *np.array(polygon.boundary.coords).flat,
                    close = False,
                    fill = 'transparent',
                    stroke = self._CAT_COLORS.get(category, '#FFFFFF'),
                )
            )

        for polygon, category in zip(
            svg_coords.geoms,
            self._room_categories
        ):
            is_horizontal = (polygon.bounds[2] - polygon.bounds[0]) >= (polygon.bounds[3] - polygon.bounds[1])

            drawing.append(
                drawsvg.Text(
                    self._CAT_LABELS.get(category, 'UNKNOWN'),
                    font_size = 6,
                    x = polygon.centroid.x,
                    y = polygon.centroid.y,
                    text_anchor = 'middle',
                    dominant_baseline = 'middle',
                    fill = self._CAT_COLORS.get(category, '#FFFFFF'),
                    transform = f'rotate({0 if is_horizontal else -90}, {polygon.centroid.x}, {polygon.centroid.y})'
                )
            )

        return drawing

    def __init__(self, floorplan, crop=True, gridmap=None, map_resolution:float=1) -> None:

        self._resolution = floorplan['resolution']
        self._room_categories = [room['category'] for room in floorplan['rooms'] if room['category'] not in (0,)]
        self._rooms = shapely.MultiPolygon([
            shapely.Polygon(room['corners']).normalize()
            for room
            in floorplan['rooms']
            if room['category'] not in (0,)
        ])
        self._doors = shapely.MultiPolygon([
            shapely.Polygon(door['corners'])
            for door
            in floorplan['doors']
            if door['category']
            not in (11,13)
        ])

        if crop:
            bounds = self._rooms.bounds
            self._resolution = (int(bounds[3]-bounds[1]), int(bounds[2]-bounds[0]))
            anchor = np.array([bounds[0], bounds[1]])

            self._rooms = shapely.transform(self._rooms, lambda x: x-anchor)
            self._doors = shapely.transform(self._doors, lambda x: x-anchor)


        if gridmap is None: gridmap = np.zeros(self._resolution)
        self.gridmap = self.compute_gridmap(gridmap)
        self.zones = self._compute_zones(map_resolution)
        self.zones_visu = self._visualize_zones()
        


@MapGeneratorFactory.register(MapGenerators.AIRCHITECT)
class AIrchitectMapGenerator(BaseMapGenerator):

    prompt: str
    _preload: typing.Optional[typing.Dict]

    def __init__(
        self,
        prompt: str = '',
        **kwargs
    ):

        super().__init__(**kwargs) 

        self.prompt = prompt
        self._preload = None

    def update_params(self, height: int, width: int, map_res: float, prompt: str, **kwargs):

        if self.prompt != prompt:
            self.prompt = prompt
            self._preload = None

        super().update_params(height, width, map_res, prompt, **kwargs)

    def retrieve_params(self):

        params = super().retrieve_params()
        prompt = str(rospy.get_param(
            MAP_GENERATOR_NS("algorithm_config/prompt"), self.prompt
        ))

        return *params, prompt

    def _load(self) -> typing.Dict:
        class Config:
            class Room:
                ID = (f'{i}' for i in itertools.count())
                Types = ['Living Room', 'Kitchen', 'Bedroom', 'Bathroom', 'Balcony', 'Entrance', 'Dining Room', 'Study Room', 'Storage']

            class Edge:
                ID = (f'{i}' for i in  itertools.count())

        def Node(room_type = None, corners = None):
            if room_type is None or room_type not in Config.Room.Types: room_type = np.random.choice(Config.Room.Types)
            if corners is None: corners = int(np.random.choice([4,6], p=[1.,0.]))
            return {
                "id": next(Config.Room.ID), # Nodes unique id   
                "room_type": room_type, # Room Type in ['Living Room', 'Kitchen', 'Bedroom', 'Bathroom', 'Balcony', 'Entrance', 'Dining Room', 'Study Room', 'Storage', 'Front Door', 'Unknown', 'Interior Door']
                "corners": corners # Number of Corners
            }
        
        rooms = [Node(*conf) for conf in int(self.prompt)*[()]]
        edges = []

        import igraph as ig

        g = ig.Graph.Full(len(rooms))
        weights = list(range(g.ecount()))
        np.random.shuffle(weights)
        g.es["weight"] = weights

        for edge in g.spanning_tree(weights=g.es["weight"], return_tree=False):
            a,b = g.es[edge].source, g.es[edge].target
            if a<b:
                edges.append({
                    "id": next(Config.Edge.ID), # Edge Unique id
                    "source": rooms[a]['id'], # First Nodes id
                    "target": rooms[b]['id']  # Second Nodes id
                })

        payload = {
            "nodes": rooms,
            "edges": edges,
            "metrics": False # If True provides the length and width of rooms in pixels
        }

        return requests.post('http://0.0.0.0:8000/generate', json=payload).json()

    def generate_grid_map(self):
        super().generate_grid_map()

        floorplan = self._preload
        self._preload = None
        if floorplan is None: floorplan = self._load()

        computed = Floorplan(
            floorplan['floorplan'][0][0],
            crop=False,
            gridmap=np.zeros((self.height, self.width)),
            map_resolution=self.map_resolution
        )

        extras = {}

        import urllib.request
        svg = urllib.request.urlopen(floorplan['dataUri'][0])
        extras['artifacts/floorplan.svg'] = svg.read()

        extras['map/zones.yaml'] = json.dumps(computed.zones, indent=4)

        extras['artifacts/zones.svg'] = computed.zones_visu.as_svg()

        return computed.gridmap, extras
    
    def idle(self):
        self._preload = self._load()
        

def test(prompt):
    map_gen = AIrchitectMapGenerator(
        height=70, width=50, map_resolution=0.25, prompt=prompt
    )
    grid_map = map_gen.generate_grid_map()


if __name__ == "__main__":
    test(sys.argv[1] if len(sys.argv) >= 2 else '5')
