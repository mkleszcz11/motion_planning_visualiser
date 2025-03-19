import random
import math
from core.algorithm import Algorithm
from core.node import Node

class RandomWalkAlgorithm(Algorithm):
    def __init__(self, map, benchmark_manager=None):
        super().__init__(map = map,
                         benchmark_manager = benchmark_manager)
        if map.start:
            start_node = Node(map.start.x, map.start.y)
            self.nodes.append(start_node)

    def step(self):
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_benchmark()

        if self.is_complete():
            #Simplified
            self.reconstruct_path()
            self.finalize_benchmark()
            return

        if not self.nodes:
            return

        last_node = self.nodes[-1]
        new_x = last_node.x + random.uniform(-self.step_size, self.step_size)
        new_y = last_node.y + random.uniform(-self.step_size, self.step_size)

        new_x = max(0, min(self.map.width, new_x))
        new_y = max(0, min(self.map.height, new_y))

        if not self.is_collision(new_x, new_y) and not self.is_edge_collision(last_node.x, last_node.y, new_x, new_y):
            new_node = Node(new_x, new_y, last_node)
            last_node.add_child(new_node)
            self.nodes.append(new_node)
            self.steps += 1

            if self.is_complete():
                self.reconstruct_path()
                self.finalize_benchmark()

    def reconstruct_path(self):
        """Reconstructs the path from the goal to the start node."""
        if self.map.goal is None:
            return

        self.path = []  # Store the path in self.path
        #  RandomWalk doesn't guarantee a node *exactly* at the goal.
        nearest_node = self.get_nearest_node((self.map.goal.x, self.map.goal.y))

        if nearest_node is None:
            return

        node = nearest_node
        while node is not None:
            self.path.append(node)
            node = node.parent
        self.path.reverse()