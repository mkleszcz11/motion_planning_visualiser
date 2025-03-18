from maps.map_config import MapConfig

from maps.maps.simple_map import simple_map
from maps.maps.simple_map_2 import simple_map_2
from maps.maps.narrow_passage import narrow_passage

class MapsManager:
    def __init__(self):
        self.maps = [
            {"name": "Simple Map", "map": simple_map},
            {"name": "Simple Map V2", "map": simple_map_2},
            {"name": "Narrow Passage", "map": narrow_passage}
        ]

    def get_map_names(self):
        return [map_obj["name"] for map_obj in self.maps]

    def get_map(self, name):
        for map_obj in self.maps:
            if map_obj["name"] == name:
                return map_obj["map"]
        return None
