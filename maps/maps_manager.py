from maps.map_config import MapConfig

from maps.maps.simple_map import simple_map
from maps.maps.simple_map_2 import simple_map_2
from maps.maps.narrow_passage import narrow_passage
from maps.maps.rooms import rooms
from maps.maps.maze import maze
from maps.maps.cluttered import cluttered
from maps.maps.open_space import open_space
from maps.maps.zig_zag import zig_zag
from maps.maps.island import island
from maps.maps.dense_map import map_dense

class MapsManager:
    def __init__(self):
        self.maps = [
            {"name": "Simple Map V1"  , "map": simple_map},
            {"name": "Simple Map V2"  , "map": simple_map_2},
            {"name": "Rooms"          , "map": rooms},
            {"name": "Dense Obstacles", "map": map_dense},
            {"name": "Narrow Passage" , "map": narrow_passage},
            {"name": "Maze Map", "map": maze},
            {"name": "Cluttered Map", "map": cluttered},
            {"name": "Open Space", "map": open_space},
            {"name": "Zig-Zag Map", "map": zig_zag},
            {"name": "Island Map", "map": island}
        ]

    def get_map_names(self):
        return [map_obj["name"] for map_obj in self.maps]

    def get_map(self, name) -> MapConfig:
        for map_obj in self.maps:
            if map_obj["name"] == name:
                return map_obj["map"]
        return None