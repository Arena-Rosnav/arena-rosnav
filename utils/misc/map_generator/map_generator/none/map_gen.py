import os
from PIL import Image
import numpy as np
import yaml

from map_generator.base_map_gen import BaseMapGenerator
from map_generator.factory import MapGeneratorFactory
from map_generator.constants import (
    ROSNAV_MAP_FOLDER,
    CellValue,
    MapGenerators,
    MAP_GENERATOR_NS,
)

import sys

import rospy


@MapGeneratorFactory.register(MapGenerators.NONE)
class NoneMapGenerator(BaseMapGenerator):

    dir: str

    def __init__(
        self,
        dir: str = 'empty_map',
        **kwargs
    ):

        super().__init__(**kwargs) 

        self.dir = dir

    def update_params(self, height: int, width: int, map_res: float, dir: str, **kwargs):

        super().update_params(height, width, map_res, **kwargs)

        self.dir = dir

        

    def retrieve_params(self):

        params = super().retrieve_params()
        dir = str(rospy.get_param(
            MAP_GENERATOR_NS("algorithm_config/dir"), self.dir
        ))

        return *params, dir


    def generate_map(self):
        super().generate_map()

        base_folder = ROSNAV_MAP_FOLDER / self.dir

        with open(base_folder / 'map' / 'map.yaml') as f:
            map_yaml = yaml.load(f, yaml.SafeLoader)

        self.map_resolution = map_yaml['resolution']

        gridmap = np.asarray(Image.open(base_folder / 'map' / map_yaml['image'])).astype(float) / 255

        # >1 channel
        if len(gridmap.shape) > 2:
            gridmap = gridmap.mean(axis=2)

        if map_yaml.get('negate', False):
            gridmap = 1 - gridmap

        occ = gridmap > map_yaml['occupied_thresh']
        fre = gridmap < map_yaml['free_thresh']

        gridmap[:,:] = -1
        gridmap[occ] = CellValue.FULL
        gridmap[fre] = CellValue.EMPTY
        #TODO switch btw ternary mode and continuous mode

        extras = {}

        for root, dirs, files in os.walk(base_folder):
            for file in (os.path.relpath(os.path.join(root,file), base_folder) for file in files):
                try:
                    with open(os.path.join(base_folder, file), 'r') as f:
                        content = f.read()
                except UnicodeDecodeError:
                    with open(os.path.join(base_folder, file), 'rb') as f:
                        content = f.read()

                extras[file] = content

        return gridmap, extras

def test(prompt):
    map_gen = AIrchitectMapGenerator(
        height=70, width=50, map_resolution=0.25, prompt=prompt
    )
    grid_map = map_gen.generate_map()


if __name__ == "__main__":
    test(sys.argv[1] if len(sys.argv) >= 2 else '5')
