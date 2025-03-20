from maps.map_config import MapConfig

maze = MapConfig(
    name='Maze Map',
    width=100,
    height=100,
    default_start=(5, 5),
    default_goal=(95, 95),
    obstacles=[
        (10, 0, 5, 40),
        (20, 60, 5, 40),
        (30, 0, 5, 40),
        (40, 60, 5, 40),
        (50, 0, 5, 40),
        (60, 60, 5, 40),
        (70, 0, 5, 40),
        (80, 60, 5, 40),
        (90, 0, 5, 40)
    ]
)

def register_map():
    return maze