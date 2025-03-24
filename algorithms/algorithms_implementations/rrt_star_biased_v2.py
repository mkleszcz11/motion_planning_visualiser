import random
import math

from typing import List

from core.algorithm import Algorithm
from core.node import Node
from core.map import Map
from benchmarks.benchmark_manager import BenchmarkManager

BIAS = 1/5

class RRTStarBiasedV2Algorithm(Algorithm):
    def __init__(self, map: Map, benchmark_manager: BenchmarkManager = None):
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

        # There is no collision of node and obstacle
        if new_node and not self.is_collision(new_node.x, new_node.y):
            if not self.is_edge_collision(nearest_node.x, nearest_node.y, new_node.x, new_node.y):
                nearest_node.add_child(new_node)
                self.nodes.append(new_node)
                self.steps += 1
                
                self.rewire_tree(new_node)

                if self.is_complete():
                    self.reconstruct_path()
                    self.finalize_benchmark()

    def get_random_sample(self):
        # Introduce goal bias
        if self.map.goal and random.random() < BIAS:
            return (self.map.goal.x, self.map.goal.y)
        else:
            return (random.uniform(0, self.map.width), random.uniform(0, self.map.height))

    def extend_toward(self, from_node: Node, to_position: tuple):
        dist = self.distance(from_node.get_position(), to_position)
        if dist < self.step_size:
            return Node(to_position[0], to_position[1], from_node)
        else:
            theta = math.atan2(to_position[1] - from_node.y, to_position[0] - from_node.x)
            new_x = from_node.x + self.step_size * math.cos(theta)
            new_y = from_node.y + self.step_size * math.sin(theta)
            return Node(new_x, new_y, from_node)

    def rewire_tree(self, new_node: Node):
        radius = self.step_size * 3 # TODO -> might be calculated in a fancy way
        nodes_to_rewire = self.get_near_nodes(new_node, radius)
        for node in nodes_to_rewire:
            if not self.is_edge_collision(node.x, node.y, new_node.x, new_node.y):
                if node.cost + self.distance(node.get_position(), new_node.get_position()) < new_node.cost:
                    new_node.parent = node
                    new_node.cost = node.cost + self.distance(node.get_position(), new_node.get_position())

    def get_near_nodes(self, node: Node, radius: float) -> List[Node]:
        near_nodes = []
        for potential_node in self.nodes:
            if potential_node == node:
                continue
            if self.distance(potential_node.get_position(), node.get_position()) < radius:
                near_nodes.append(potential_node)
        return near_nodes
        