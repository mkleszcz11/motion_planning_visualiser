from maps.map_config import MapConfig

simple_map = MapConfig(
    name='Simple Map',
    width=100,  # KEEP IT AS 100
    height=100, # KEEP IT AS 100
    obstacles=[
        (0, 0, 5, 5),   # Top-left corner
        (95, 0, 5, 5),  # Top-right corner
        (0, 95, 5, 5),  # Bottom-left corner
        (95, 95, 5, 5), # Bottom-right corner
        (10, 10, 5, 5),
        (50, 20, 10, 5),
        (30, 70, 15, 5),
        (80, 10, 5, 5),
        (80, 80, 10, 10)
    ]
)

def register_map():
    return simple_map
