from maps.map_config import MapConfig

open_space = MapConfig(
    name='Open Space',
    width=100,
    height=100,
    default_start=(5, 5),
    default_goal=(95, 95),
    obstacles=[
        (20, 20, 10, 10), 
        (50, 50, 15, 15),
        (70, 30, 10, 10)
    ]
)

def register_map():
    return open_space
