import random
import math
from core.algorithm import Algorithm
from core.node import Node


class HybridSamplingPRMStarAlgorithm(Algorithm):
    def __init__(self, map, benchmark_manager=None, uniform_percentage=0.5, gaussian_std_dev=5.0):
        super().__init__(map=map, benchmark_manager=benchmark_manager)
        self.uniform_percentage = uniform_percentage
        self.gaussian_std_dev = gaussian_std_dev
        self.nodes = []  # Initialize nodes here (important for PRM)
        self.path = []

    def step(self):
        # For prm, we do not need to run in steps.
        pass

    def run_algorithm(self, num_samples):
        """Runs the complete PRM* algorithm with hybrid sampling."""
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_benchmark()

        # 1. Node Generation (using hybrid sampling)
        for _ in range(num_samples):
            q_rand = self.get_random_sample()
            if q_rand and not self.is_collision(q_rand[0], q_rand[1]):
                self.nodes.append(Node(q_rand[0], q_rand[1]))  # No parent for initial nodes

        # 2. Connection (PRM* logic)
        for node in self.nodes:
            near_nodes = self.get_near_nodes(node)
            for near_node in near_nodes:
                if node != near_node and not self.is_edge_collision(node.x, node.y, near_node.x, near_node.y):
                    node.add_child(near_node)  # Bidirectional connection in PRM
                    near_node.add_child(node)

        # 3. Pathfinding (after all nodes and edges are added)
        if self.is_complete():  # Check if a path is possible
            self.reconstruct_path()
            self.finalize_benchmark()

    def get_random_sample(self):
        # Hybrid sampling:  Mix between uniform and Gaussian
        if random.random() < self.uniform_percentage:
            # Uniform sampling
            return (random.uniform(0, self.map.width), random.uniform(0, self.map.height))
        else:
            # Gaussian sampling (centered around existing nodes)
            if not self.nodes:  # Handle edge case of no existing nodes
                return (random.uniform(0, self.map.width), random.uniform(0, self.map.height))

            while True:  # Keep sampling until a collision-free sample is found
                mean_node = random.choice(self.nodes)
                x = random.gauss(mean_node.x, self.gaussian_std_dev)
                y = random.gauss(mean_node.y, self.gaussian_std_dev)

                # Constrain to map boundaries
                x = max(0, min(x, self.map.width))
                y = max(0, min(y, self.map.height))

                if not self.is_collision(x, y):
                    return (x, y)

    def get_near_nodes(self, q_new):
        radius = self.calculate_radius()
        near_nodes = []
        for node in self.nodes:
            if self.distance(node.get_position(), q_new.get_position()) < radius:
                near_nodes.append(node)
        return near_nodes

    def calculate_radius(self):
        n = len(self.nodes)
        if n == 0:
            return float('inf')
        d = 2
        gamma = self.calculate_gamma()
        radius = gamma * (math.log(n) / n) ** (1 / d)
        return radius  # No longer limiting to step_size

    def calculate_gamma(self):
        d = 2
        zeta_d = math.pi
        gamma = 2 * (1 + 1 / d) ** (1 / d) * (self.map.width * self.map.height / zeta_d) ** (1 / d)
        return gamma

    def is_complete(self):
        """Checks if a path exists between start and goal."""
        if self.map.start is None or self.map.goal is None:
            return False

        start_node = self.get_nearest_node((self.map.start.x, self.map.start.y))
        goal_node = self.get_nearest_node((self.map.goal.x, self.map.goal.y))

        if start_node is None or goal_node is None:
            return False

        # Check for connectivity using a breadth-first search (BFS)
        return self.is_connected(start_node, goal_node)

    def is_connected(self, start_node, goal_node):
        """Uses BFS to check if two nodes are in the same connected component."""
        visited = set()
        queue = [start_node]

        while queue:
            current_node = queue.pop(0)
            if current_node == goal_node:
                return True
            visited.add(current_node)
            for neighbor in current_node.children:
                if neighbor not in visited:
                    queue.append(neighbor)
        return False

    def reconstruct_path(self):
        """Reconstructs the path from start to goal using BFS (for PRM)."""
        if self.map.start is None or self.map.goal is None:
            return

        start_node = self.get_nearest_node((self.map.start.x, self.map.start.y))
        goal_node = self.get_nearest_node((self.map.goal.x, self.map.goal.y))

        if start_node is None or goal_node is None:
            return

        # Perform a Breadth-First Search (BFS) to find the shortest path
        queue = [(start_node, [start_node])]  # (node, path_to_node)
        visited = {start_node}

        while queue:
            current_node, current_path = queue.pop(0)
            if current_node == goal_node:
                self.path = current_path
                return

            for neighbor in current_node.children:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, current_path + [neighbor]))
        # If no path is found, the self.path remains an empty list.

    def cost(self, node):
        # For PRM we do not need this.
        return 0