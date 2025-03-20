from maps.map_config import MapConfig

zig_zag = MapConfig(
    name='Zig-Zag Map',
    width=100,
    height=100,
    default_start=(5, 5),
    default_goal=(95, 95),
    obstacles=[
        (10, 10, 20, 5),
        (30, 20, 20, 5),
        (50, 30, 20, 5),
        (70, 40, 20, 5),
        (30, 60, 40, 5),
        (50, 70, 20, 5),
        (70, 80, 20, 5)
    ]
)

def register_map():
    return zig_zag
