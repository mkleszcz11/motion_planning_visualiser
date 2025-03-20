class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.children = []

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

