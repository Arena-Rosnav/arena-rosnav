import itertools
import math
import typing

import cv2
import numpy as np
import requests
import rospy

from map_generator.base_map_gen import BaseMapGenerator
from map_generator.factory import MapGeneratorFactory
from map_generator.constants import (
    MapGenerators,
    MAP_GENERATOR_NS,
)

import sys

def _pairwise(iterable):
    iterator = iter(iterable)
    a = next(iterator, None)
    for b in iterator:
        yield a, b
        a = b

class Floorplan:

    lines: typing.List[
        typing.Tuple[
            typing.Tuple[int, int],
            typing.Tuple[int, int]
        ]
    ]
    gridmap: np.ndarray

    def _compute_lines(self, rooms, doors):
        lines: typing.Dict[int, typing.List] = {}
        
        for room in rooms:
            for point_from, point_to in _pairwise(room + [room[0]]):
                angle = np.angle(complex(*np.array(point_to) - np.array(point_from)))
                angle = int(np.trunc(180/np.pi * angle)) % 180
                lines.setdefault(angle, []).append((point_from, point_to))

        for door in doors:
            
            door = np.array(door)

            diagonal = np.abs(door[0]-door[2])
            is_horizontal = diagonal[0] > diagonal[1]
            long_neighbor = 1 if is_horizontal else -1

            angle = np.angle(complex(*door[long_neighbor] - door[0]))
            index_angle = int(np.trunc(180/np.pi * angle)) % 180
            target = lines.setdefault(index_angle, [])

            TF = np.array([
                [np.cos(angle), -np.sin(angle)],
                [np.sin(angle), np.cos(angle)]
            ])
            FT = np.flip(TF, (0,1)) # "anti"-transpose for right multiplying

            door = door.dot(TF)
            targets = np.array(target).dot(TF)

            long_0, short_0 = np.min([door[0], door[2]], axis=0)
            long_1, short_1 = np.max([door[0], door[2]], axis=0)
            tolerance = (short_1 - short_0)/2

            crosses_door = np.logical_and.reduce([
                -tolerance <= np.mean(targets[:,:,1], axis=1) - short_0,
                -tolerance <= short_1 - np.mean(targets[:,:,1], axis=1),
                long_0 < np.max(targets[:,:,0], axis=1),
                long_1 > np.min(targets[:,:,0], axis=1)
            ])

            for hit in reversed(np.where(crosses_door)[0]):

                to_split = np.array(target.pop(hit)).dot(TF)
                to_split = np.array(sorted(to_split.tolist(), key=lambda x: x[0]))
                
                short = np.mean(to_split[:,1]) # though short1==short2 is implicitly assumed already

                if (end:=to_split[0][0]) < long_0:
                    target.append(
                        np.array([
                            np.array([end, short]),
                            np.array([long_0, short])
                        ]).dot(FT)
                    )
            
                if (end:=to_split[1][0]) > long_1:
                    target.append(
                        np.array([
                            np.array([long_1, short]),
                            np.array([end, short])
                        ]).dot(FT)
                    )
       
        self.lines = [
            (
                (int(line[0][0]), int(line[0][1])),
                (int(line[1][0]), int(line[1][1])),
            )
            for line
            in itertools.chain(*lines.values())
        ]

    def _draw_gridmap(self, lines, resolution):
        canvas = np.zeros(resolution)
        for point_from, point_to in lines:
            cv2.line(canvas, point_from, point_to, 1, 1)
        
        self.gridmap = canvas

    def _compute_zones(self):
        ...

    def __init__(self, floorplan, crop=False):

        resolution = (floorplan['resolution'], floorplan['resolution'])
        rooms = [room['corners'] for room in floorplan['rooms'] if room['category'] not in (0,)]
        doors = [door['corners'] for door in floorplan['doors'] if door['category'] not in (11,13)]

        if crop:
            vertices = np.array(rooms)
            min_x, min_y = vertices.reshape((-1,2)).min(axis=0)
            max_x, max_y = vertices.reshape((-1,2)).max(axis=0)

            anchor =  np.array([min_x, min_y])

            rooms = vertices - anchor[None, :]
            doors = np.array(doors) - anchor[None, :]
            resolution = (math.ceil(max_y - min_y), math.ceil(max_x - min_x))

        self._compute_lines(
            rooms,
            doors,
        )

        self._draw_gridmap(self.lines, resolution)

        self._compute_zones()


@MapGeneratorFactory.register(MapGenerators.AIRCHITECT)
class AIrchitectMapGenerator(BaseMapGenerator):

    prompt: str
    _preload: typing.Optional[str]

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

    def _load(self) -> str:
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

        return requests.post('http://0.0.0.0:8000/generate', json=payload).json()['floorplan'][0][0]

    def generate_grid_map(self) -> np.ndarray:
        super().generate_grid_map()

        floorplan = self._preload
        self._preload = None
        if floorplan is None: floorplan = self._load()

        rospy.logwarn('loaded')
        
        computed = Floorplan(floorplan, crop=True)
        gridmap = computed.gridmap

        import cv2

        if self.width/gridmap.shape[1] < self.height/gridmap.shape[0]:
            gridmap = cv2.resize(gridmap, dsize=(int(self.width/gridmap.shape[1] * gridmap.shape[0]), self.width), interpolation=cv2.INTER_NEAREST)
            to_pad = self.width-gridmap.shape[1]
            gridmap = cv2.copyMakeBorder(gridmap, 0, 0, to_pad//2, to_pad//2 + to_pad%2, cv2.BORDER_CONSTANT, value=[1])
        else:
            gridmap = cv2.resize(gridmap, dsize=(self.height, int(self.height/gridmap.shape[0] * gridmap.shape[1])), interpolation=cv2.INTER_NEAREST)
            to_pad = self.height-gridmap.shape[0]
            gridmap = cv2.copyMakeBorder(gridmap, to_pad//2, to_pad//2 + to_pad%2, 0, 0, cv2.BORDER_CONSTANT, value=[1])
        
        return gridmap
    
    def idle(self):
        self._preload = self._load()
        

def test(prompt):
    map_gen = AIrchitectMapGenerator(
        height=70, width=50, map_resolution=0.25, prompt=prompt
    )
    grid_map = map_gen.generate_grid_map()


if __name__ == "__main__":
    test(sys.argv[1] if len(sys.argv) >= 2 else '5 rooms')
