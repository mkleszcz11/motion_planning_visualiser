class MapConfig:
    def __init__(self, name, width, height, obstacles, default_start=(10, 10), default_goal=(90, 90)):
        self.name = name
        self.width = width
        self.height = height
        self.default_start = default_start
        self.default_goal = default_goal
        self.obstacles = obstacles
