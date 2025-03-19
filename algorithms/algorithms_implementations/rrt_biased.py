import random
import math
from core.algorithm import Algorithm
from core.node import Node

BIAS = 0.3

class RRTBiasedAlgorithm(Algorithm):
    def __init__(self, map, benchmark_manager=None):
        super().__init__(map = map,
                         benchmark_manager = benchmark_manager)
        if map.start:
            # Directly reference Node attributes
            start_node = Node(map.start.x, map.start.y)
            self.nodes.append(start_node)

    def step(self):
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_benchmark()

        # Sample a random point
        sample = self.get_random_sample()

        # Find nearest node in the tree
        nearest_node = self.get_nearest_node(sample)

        # Extend toward sample
        new_node = self.extend_toward(nearest_node, sample)

        if new_node and not self.is_collision(new_node.x, new_node.y):
            if not self.is_edge_collision(nearest_node.x, nearest_node.y, new_node.x, new_node.y):
                nearest_node.add_child(new_node)
                self.nodes.append(new_node)
                self.steps += 1

                if self.is_complete():
                    self.reconstruct_path()  # This now stores the path in self.path
                    self.finalize_benchmark()

    def get_random_sample(self):
        if self.map.goal and random.random() < BIAS:
            return (self.map.goal.x, self.map.goal.y)
        else:
            return (random.uniform(0, self.map.width), random.uniform(0, self.map.height))

    def extend_toward(self, from_node, to_position):
        dist = self.distance(from_node.get_position(), to_position)
        if dist < self.step_size:
            return Node(to_position[0], to_position[1], from_node)
        else:
            theta = math.atan2(to_position[1] - from_node.y, to_position[0] - from_node.x)
            new_x = from_node.x + self.step_size * math.cos(theta)
            new_y = from_node.y + self.step_size * math.sin(theta)
            return Node(new_x, new_y, from_node)

    def reconstruct_path(self):
        """Reconstructs the path from the goal to the start node."""
        if self.map.goal is None:
            return

        self.path = []  # Store the path in self.path
        node = self.get_nearest_node((self.map.goal.x, self.map.goal.y))

        while node is not None:
            self.path.append(node)
            node = node.parent
        self.path.reverse()  # Reverse to get start -> goal order