from maps.map_config import MapConfig

maze = MapConfig(
    name='Maze Map',
    width=100,
    height=100,
    default_start=(30, 50),
    default_goal=(70, 50),
    obstacles=[
        # Enclosure around start, open only to the left
        (10, 40, 30, 1),
        (10, 59, 30, 1),
        (40, 40, 1, 20),
        
        # Enclosure around goal, open only to the right
        (60, 40, 30, 1),
        (60, 59, 30, 1),
        (60, 40, 1, 20),
        
        # Add two obstacles in the middle
        (49, 10, 1, 30),
        (51, 60, 1, 30)
    ]
)

def register_map():
    return maze