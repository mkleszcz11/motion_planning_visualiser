from maps.map_config import MapConfig

cluttered = MapConfig(
    name='Cluttered Map',
    width=100,
    height=100,
    default_start=(5, 5),
    default_goal=(95, 95),
    obstacles=[
        (10, 10, 5, 5), (20, 20, 5, 5), (30, 30, 5, 5),
        (40, 40, 5, 5), (50, 50, 5, 5), (60, 60, 5, 5),
        (70, 70, 5, 5), (80, 80, 5, 5), (90, 90, 5, 5),
        (15, 85, 5, 5), (25, 75, 5, 5), (35, 65, 5, 5),
        (45, 55, 5, 5), (55, 45, 5, 5), (65, 35, 5, 5),
        (75, 25, 5, 5), (85, 15, 5, 5)
    ]
)

def register_map():
    return cluttered