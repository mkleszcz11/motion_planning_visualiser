from core.node import TreeNode, GraphNode

class Map:
    def __init__(self, width, height, start=None, goal=None, architecture="tree"):
        self.width = float(width)
        self.height = float(height)
        self.start = start
        self.goal = goal
        self.obstacles = []
        self.architecture = architecture

    def set_start(self, x, y):
        if self.architecture == "tree":
            self.start = TreeNode(float(x), float(y), None)
        elif self.architecture == "graph":
            self.start = GraphNode(float(x), float(y))
        else:
            raise ValueError("Unknown architecture")

    def set_goal(self, x, y):
        if self.architecture == "tree":
            self.goal = TreeNode(float(x), float(y), None)
        elif self.architecture == "graph":
            self.goal = GraphNode(float(x), float(y))
        else:
            raise ValueError("Unknown architecture")

    def add_obstacle(self, x, y, width, height):
        self.obstacles.append((float(x), float(y), float(width), float(height)))

    def reset(self):
        self.start = None
        self.goal = None
        self.obstacles = []

    def get_obstacles(self):
        return self.obstacles
