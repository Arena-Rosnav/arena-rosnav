import itertools

import typing
import cv2
import numpy as np
import rospy

import requests
import shapely

from map_generator.base_map_gen import BaseMapGenerator
from map_generator.factory import MapGeneratorFactory
from map_generator.constants import (
    MapGenerators,
    MAP_GENERATOR_NS,
)

import sys

class Floorplan:
    def __init__(self, floorplan, crop=True, gridmap=None) -> None:
        import shapely

        resolution = (floorplan['resolution'], floorplan['resolution'])
        rooms = shapely.MultiPolygon([
            shapely.Polygon(room['corners']).normalize()
            for room
            in floorplan['rooms']
            if room['category'] not in (0,)
        ])
        doors = shapely.MultiPolygon([
            shapely.Polygon(door['corners'])
            for door
            in floorplan['doors']
            if door['category']
            not in (11,13)
        ])

        if crop:
            bounds = rooms.bounds
            resolution = (int(bounds[3]-bounds[1]), int(bounds[2]-bounds[0]))
            anchor = np.array([bounds[0], bounds[1]])

            rooms = shapely.transform(rooms, lambda x: x-anchor)
            doors = shapely.transform(doors, lambda x: x-anchor)


        if gridmap is None: gridmap = np.zeros(resolution)
        
        transform = np.eye(2)

         # orient both vertical
        if (gridmap_flipped := gridmap.shape[0] < gridmap.shape[1]):
            gridmap = gridmap.T

        if resolution[0] < resolution[1]:
            resolution = resolution[::-1]
            transform = np.array([[0,1],[1,0]]) @ transform

        scaling = min(1, gridmap.shape[0] / resolution[0], gridmap.shape[1] / resolution[1])

        transform = np.array([[scaling, 0], [0, scaling]])

        rooms = shapely.transform(rooms, lambda x: x.dot(transform))
        doors = shapely.transform(doors, lambda x: x.dot(transform))
        
        canvas = np.zeros(resolution, dtype=np.int32)

        for room in rooms.geoms:
            cv2.fillConvexPoly(
                canvas,
                np.array(room.boundary.coords).astype(np.int32),
                (0,)
            )

        for room in rooms.geoms:
            cv2.polylines(
                canvas,
                [np.array(room.boundary.coords).astype(np.int32).reshape((-1,1,2))],
                False,
                (1,),
                1
            )

        for door in doors.geoms:
            cv2.fillConvexPoly(
                canvas,
                np.array(door.boundary.coords).astype(np.int32),
                (0,)
            )

        gridmap[:,:] = 1
        gridmap[:canvas.shape[0], :canvas.shape[1]] = canvas

        if gridmap_flipped:
            gridmap = gridmap.T

        self.gridmap = gridmap

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
            crop=True,
            gridmap=np.zeros((self.height, self.width))
        )

        extras = {}

        import urllib.request
        svg = urllib.request.urlopen(floorplan['dataUri'][0])
        extras['artifacts/floorplan.svg'] = svg.read()

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
