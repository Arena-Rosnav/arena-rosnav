import itertools
import typing

import math
import urllib.request
import cv2
import numpy as np

def _pairwise(iterable):
    iterator = iter(iterable)
    a = next(iterator, None)
    for b in iterator:
        yield a, b
        a = b


class Floorplan:

    _lines: typing.List[
        typing.Tuple[
            typing.Tuple[int, int],
            typing.Tuple[int, int]
        ]
    ]
    _gridmap: np.ndarray

    def _compute_lines(self, rooms, doors):
        lines: typing.Dict[int, typing.List] = {}
        
        for room in rooms.copy():
            for point_from, point_to in _pairwise(np.concatenate([room, [room[0]]])):
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

            rejoins_left = []
            rejoins_right = []

            for hit in reversed(np.where(crosses_door)[0]):

                to_split = np.array(target.pop(hit)).dot(TF)
                to_split = np.array(sorted(to_split.tolist(), key=lambda x: x[0]))
                
                short = np.mean(to_split[:,1]) # though short1==short2 is implicitly assumed already

                if (end:=to_split[0][0]) < long_0:
                    target.append(
                        np.array([
                            [end, short],
                            [long_0, short]
                        ]).dot(FT)
                    )
                    rejoins_left.append(short)
            
                if (end:=to_split[1][0]) > long_1:
                    target.append(
                        np.array([
                            [long_1, short],
                            [end, short]
                        ]).dot(FT)
                    )
                    rejoins_right.append(short)
       
            for i, first in enumerate(rejoins_left):
                for second in rejoins_left[i+1:]:
                    if first != second:
                        target.append(
                            np.array([
                                [long_0, first],
                                [long_0, second]
                            ]).dot(FT)
                        )

            for i, first in enumerate(rejoins_right):
                for second in rejoins_right[i+1:]:
                    if first != second:
                        target.append(
                            np.array([
                                [long_1, first],
                                [long_1, second]
                            ]).dot(FT)
                        )

        self._lines = [
            (
                (int(line[0][0]), int(line[0][1])),
                (int(line[1][0]), int(line[1][1])),
            )
            for line
            in itertools.chain(*lines.values())
        ]

    def _draw_gridmap(self, lines):
        
        for point_from, point_to in lines:
            cv2.line(
                self._gridmap,
                self.transform(point_from),
                self.transform(point_to),
                (1,),
                1
            )

        return self._gridmap

    def _compute_zones(self, rooms):

        room_label = {
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

        self._zones = []
        for polygon, category in rooms:
            self._zones.append({
                'label': room_label[category],
                'category': [room_label[category].lower()], #todo
                'polygon': [self.transform(vertex) for vertex in polygon]
            })

        return self._zones

    def transform(self, point):
        return [
            int(coord)
            for coord in
            list((self._transform_matrix @ np.array([*point, 1])).flat)[:2]
        ]

    def __init__(self, floorplan, crop=False, canvas = None):

        self._resolution = (floorplan['resolution'], floorplan['resolution'])
        self._rooms = np.array([room['corners'] for room in floorplan['rooms'] if room['category'] not in (0,)])
        self._doors = np.array([door['corners'] for door in floorplan['doors'] if door['category'] not in (11,13)])

        if crop:
            vertices = np.array(self._rooms)
            min_x, min_y = vertices.min(axis=(0,1))
            max_x, max_y = vertices.max(axis=(0,1))

            anchor =  np.array([min_x, min_y])

            self._rooms = vertices - anchor
            self._doors = np.array(self._doors) - anchor
            self._resolution = (math.ceil(max_x - min_x), math.ceil(max_y - min_y))

        if canvas is None: canvas = np.zeros(self._resolution)
        self._gridmap = canvas

        self._transform_matrix = np.eye(3)

        # orient both landscape
        if self._gridmap.shape[1] < self._gridmap.shape[0]:
            self._gridmap = self._gridmap.T
            self._transform_matrix = np.matrix([
                [0,1,0],
                [1,0,0],
                [0,0,0]
            ]) @ self._transform_matrix

        if self._resolution[0] < self._resolution[1]:
            self._resolution = self._resolution[::-1]
            self._transform_matrix = np.matrix([
                [0,1,0],
                [1,0,0],
                [0,0,0]
            ]) @ self._transform_matrix

        assert self._gridmap.shape[1] >= self._gridmap.shape[0]
        assert self._resolution[0] >= self._resolution[1]

        scaling = self._gridmap.shape[1] / self.resolution[0]
        to_pad = math.ceil(self._gridmap.shape[0] - scaling * self.resolution[1])

        self._gridmap[:,:] = 1

        # affine transform
        self._transform_matrix = np.array([
            [scaling, 0, to_pad//2],
            [0, scaling, 0],
            [0, 0, 1]
        ]) @ self._transform_matrix

        self._compute_lines(
            self._rooms,
            self._doors,
        )

        self._compute_zones(zip(self._rooms.tolist(), (room['category'] for room in floorplan['rooms'] if room['category'] not in (0,))))

        for zone in self._zones:
            cv2.fillConvexPoly(
                self._gridmap,
                np.array(zone['polygon']),
                (0,)
            )

        self._draw_gridmap(self._lines)

    @property
    def resolution(self) -> typing.Tuple[int, int]:
        return self._resolution
    
    @property
    def gridmap(self) -> np.ndarray:
        return self._gridmap