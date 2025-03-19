class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.children = []

    def __str__(self):
        return f"Node at ({self.x}, {self.y})"

    def add_child(self, child):
        self.children.append(child)

    def get_position(self):
        return (self.x, self.y)
