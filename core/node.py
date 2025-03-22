import math

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = float("inf")
        self.children = []

        self.calculate_cost()

    def __str__(self):
        return f"Node at ({self.x}, {self.y})"

    def get_position(self):
        return (self.x, self.y)

    def add_child(self, child):
        self.children.append(child)

    def remove_child(self, child_node):
        """Removes a child from the node's children list, handling the case
        where the child might not be present.
        """
        if child_node in self.children:  # IMPORTANT: Check for presence
            self.children.remove(child_node)

    def calculate_cost(self):
        if self.parent is not None:
            self.cost = self.parent.cost + math.sqrt((self.x - self.parent.x) ** 2 + (self.y - self.parent.y) ** 2)
        else:
            self.cost = 0.0 # No parent means it is the root node
