import random
import math
from core.algorithm import Algorithm
from core.node import TreeNode

BIAS = 0.3

class RRTBiasedAlgorithm(Algorithm):
    def __init__(self, map, benchmark_manager=None):
        super().__init__(map = map,
                         benchmark_manager = benchmark_manager)
        if map.start:
            # Directly reference TreeNode attributes
            start_node = TreeNode(map.start.x, map.start.y)
            goal_node = TreeNode(map.goal.x, map.goal.y)
            self.nodes.append(start_node)
            self.start_node = start_node
            self.goal_node = goal_node

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
                    self.reconstruct_path()
                    self.finalize_benchmark()

    def get_random_sample(self):
        if self.map.goal and random.random() < BIAS:
            return (self.map.goal.x, self.map.goal.y)
        else:
            return (random.uniform(0, self.map.width), random.uniform(0, self.map.height))

    def extend_toward(self, from_node, to_position):
        dist = self.distance(from_node.get_position(), to_position)
        if dist < self.step_size:
            return TreeNode(to_position[0], to_position[1], from_node)
        else:
            theta = math.atan2(to_position[1] - from_node.y, to_position[0] - from_node.x)
            new_x = from_node.x + self.step_size * math.cos(theta)
            new_y = from_node.y + self.step_size * math.sin(theta)
            return TreeNode(new_x, new_y, from_node)
