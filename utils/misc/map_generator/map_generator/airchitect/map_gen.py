import io
import itertools
from typing import Tuple
import typing

import PIL
import PIL.Image
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

        return requests.post('http://0.0.0.0:8000/generate', json=payload).json()['dataUri'][0]

    def generate_grid_map(self) -> np.ndarray:
        super().generate_grid_map()

        svg = self._preload
        self._preload = None
        if svg is None: svg = self._load()
        
        import cairosvg

        img = io.BytesIO()
        cairosvg.svg2png(url=svg, write_to=img, scale=5)
        gray = np.array(PIL.Image.open(img)).mean(axis=-1).astype(int)

        occupied = np.argwhere(gray != 255)
        top_left = occupied.min(axis=0)
        bottom_right = occupied.max(axis=0)
        gray = gray[top_left[0]:bottom_right[0]+1, top_left[1]:bottom_right[1]+1]
        
        gray = np.array(gray == 0).astype(int)

        if (self.width > self.height) ^ (gray.shape[1] > gray.shape[0]):
            gray = gray.T

        import cv2

        if self.width/gray.shape[1] < self.height/gray.shape[0]:
            gray = cv2.resize(gray, dsize=(int(self.width/gray.shape[1] * gray.shape[0]), self.width), interpolation=cv2.INTER_NEAREST)
            to_pad = self.width-gray.shape[1]
            gray = cv2.copyMakeBorder(gray, 0, 0, to_pad//2, to_pad//2 + to_pad%2, cv2.BORDER_CONSTANT, value=[1])
        else:
            gray = cv2.resize(gray, dsize=(self.height, int(self.height/gray.shape[0] * gray.shape[1])), interpolation=cv2.INTER_NEAREST)
            to_pad = self.height-gray.shape[0]
            gray = cv2.copyMakeBorder(gray, to_pad//2, to_pad//2 + to_pad%2, 0, 0, cv2.BORDER_CONSTANT, value=[1])
            
        return gray
    
    def idle(self):
        rospy.logwarn("idling")
        self._preload = self._load()
        

def test(prompt):
    map_gen = AIrchitectMapGenerator(
        height=70, width=50, map_resolution=0.25, prompt=prompt
    )
    grid_map = map_gen.generate_grid_map()


if __name__ == "__main__":
    test(sys.argv[1] if len(sys.argv) >= 2 else '5 rooms')
