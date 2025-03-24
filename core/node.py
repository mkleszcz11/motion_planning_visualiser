from abc import ABC
import math

class Node(ABC):
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def __str__(self):
        return f"Node at ({self.x}, {self.y})"

    def get_position(self):
        return (self.x, self.y)

class TreeNode(Node):
    def __init__(self, x, y, parent=None):
        super().__init__(x, y)
        self.parent = parent
        self.cost = float("inf")
        self.children = []

        self.calculate_cost()

    def add_child(self, child):
        self.children.append(child)

    def remove_child(self, child_node):
        """Removes a child from the node's children list, handling the case
        where the child might not be present.
        """
        if child_node in self.children:
            self.children.remove(child_node)

    def calculate_cost(self):
        if self.parent is not None:
            self.cost = self.parent.cost + math.sqrt((self.x - self.parent.x) ** 2 + (self.y - self.parent.y) ** 2)
        else:
            self.cost = 0.0  # No parent means it is the root node

class GraphNode(Node):
    def __init__(self, x, y):
        super().__init__(x, y)
        self.cost = float("inf")
        self.edges = {}  # {node: cost}

    def add_edge(self, node, cost):
        """Create a bidirectional edge."""
        self.edges[node] = cost
        node.edges[self] = cost

    def remove_edge(self, node):
        """Remove a bidirectional edge."""
        if node in self.edges:
            del self.edges[node]
        if self in node.edges:
            del node.edges[self]
