from maps.map_config import MapConfig

rooms = MapConfig(
    name='Rooms Map',
    width=100,
    height=100,
    default_start=(5, 32),
    default_goal=(95, 32),
    obstacles=[
        (0, 0, 30, 30),   # Top-left room
        (70, 0, 30, 30),  # Top-right room
        (0, 70, 30, 30),  # Bottom-left room
        (70, 70, 30, 30), # Bottom-right room
        (30, 15, 10, 2),  # Doorway 1
        (60, 15, 10, 2),  # Doorway 2
        (15, 30, 2, 10),  # Doorway 3
        (15, 60, 2, 10), # Doorway 4
        (85, 30, 2, 10),  # Doorway 5
        (85, 60, 2, 10), # Doorway 6
        (30, 85, 10, 2),  # Doorway 7
        (60, 85, 10, 2)   # Doorway 8

    ]
)

def register_map():
    return rooms