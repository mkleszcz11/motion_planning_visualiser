from core.node import Node

class Map:
    def __init__(self, width, height, start=None, goal=None):
        self.width = float(width)
        self.height = float(height)
        self.start = start
        self.goal = goal
        self.obstacles = []

    def set_start(self, x, y):
        self.start = Node(float(x), float(y))  # Store as Node object

    def set_goal(self, x, y):
        self.goal = Node(float(x), float(y))  # Store as Node object

    def add_obstacle(self, x, y, width, height):
        self.obstacles.append((float(x), float(y), float(width), float(height)))

    def reset(self):
        self.start = None
        self.goal = None
        self.obstacles = []

    def get_obstacles(self):
        return self.obstacles
