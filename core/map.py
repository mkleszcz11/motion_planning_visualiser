class Map:
    def __init__(self, width, height):
        self.width = float(width)
        self.height = float(height)
        self.start = None
        self.goal = None
        self.obstacles = []

    def set_start(self, x, y):
        self.start = (float(x), float(y))

    def set_goal(self, x, y):
        self.goal = (float(x), float(y))

    def add_obstacle(self, x, y, width, height):
        self.obstacles.append((float(x), float(y), float(width), float(height)))

    def reset(self):
        self.start = None
        self.goal = None
        self.obstacles = []

    def get_obstacles(self):
        return self.obstacles
