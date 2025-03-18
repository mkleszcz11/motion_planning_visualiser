from maps.map_config import MapConfig

simple_map_2 = MapConfig(
    name='Simple Map 2',
    width=100,  # KEEP IT AS 100
    height=100, # KEEP IT AS 100
    obstacles=[
        (5, 5, 8, 8),
        (20, 10, 12, 6),
        (35, 15, 15, 10),
        (70, 5, 10, 8),
        (50, 50, 20, 10),
        (75, 35, 8, 12),
        (90, 10, 5, 5),
        (10, 60, 12, 12),
        (40, 70, 15, 8),
        (70, 75, 10, 10)
    ]
)

def register_map():
    return simple_map_2
