from maps.map_config import MapConfig

island = MapConfig(
    name='Island Map',
    width=100,
    height=100,
    default_start=(5, 5),
    default_goal=(95, 95),
    obstacles=[
        (20, 20, 15, 15),
        (50, 20, 15, 15),
        (80, 20, 15, 15),
        (20, 50, 15, 15),
        (50, 50, 15, 15),
        (80, 50, 15, 15),
        (20, 80, 15, 15),
        (50, 80, 15, 15),
        (80, 80, 15, 15)
    ]
)

def register_map():
    return island
