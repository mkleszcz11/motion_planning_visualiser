import random
import heapq

import numpy as np

from core.algorithm import Algorithm

from core.map import Map
from core.node import GraphNode
from benchmarks.benchmark_manager import BenchmarkManager

from core.logger import logger

class HybridPRMAlgorithm(Algorithm):
    """Hybrid Probabilistic Road Maps (PRM) algorithm implementation using A* search.
    PRM builds a roadmap of collision-free configurations and connects them.
    A* is used to find the shortest path between start and goal.

    This version is enhanced with a hybrid approach using Gaussian sampling.
    """

    def __init__(self,
                 map: Map,
                 benchmark_manager: BenchmarkManager = None,
                 num_samples_excluding_grid: int = 500,
                 radius_as_step_size_multiplication: float = 5):
        super().__init__(map=map, benchmark_manager=benchmark_manager, architecture="graph")

        self.architecture = "graph"
        self.samples = []  # Collision-free roadmap nodes (GraphNode)
        self.neighbour_radius = self.step_size * radius_as_step_size_multiplication # distance for which nodes are considered neighbors
        self.num_samples = num_samples_excluding_grid  # Total number of random samples (can be adjusted)
        self.parent_map = {}  # Used to reconstruct the path
        self.nodes_in_the_grid = 0

        # Add start and goal
        if map.start:
            self.start_node = GraphNode(map.start.x, map.start.y)
        if map.goal:
            self.goal_node = GraphNode(map.goal.x, map.goal.y)

    def step(self):
        """Executes the full PRM algorithm in one step:
        - Sample valid points
        - Build roadmap
        - Add start/goal
        - Run A* to find a path
        """
        if self.steps == 0:
            logger.info(f"Generating grid and additional {self.num_samples} samples")
            self.generate_default_grid()
            self.generate_points_on_the_map()

        elif self.steps == 1:
            logger.info("Connecting neighbors")
            for node in self.samples:
                self.connect_neighbors(node)
            self.connect_neighbors(self.start_node)
            self.connect_neighbors(self.goal_node)
            self.nodes = self.samples + [self.start_node, self.goal_node]

        elif self.steps == 2:
            # Benchmark should not take into account the time to generate the roadmap
            if self.start_time is None and self.benchmark_manager is not None:
                self.start_benchmark()
            
            logger.info("Running A*")
            self.a_star()

            logger.info("Checking if complete")
            if self.is_complete():
                self.reconstruct_path()
                self.finalize_benchmark()
            
        self.steps += 1

    def generate_points_on_the_map(self):
        """Generate random valid samples on the map using Gaussian sampling."""
        attempts = 0
        gaussian_ratio = 0.2  # Adjust the ratio of Gaussian samples
        num_gaussian_samples = int(self.num_samples * gaussian_ratio)

        while len(self.samples) - self.nodes_in_the_grid < self.num_samples and attempts < self.num_samples * 5:
            if attempts < num_gaussian_samples:
                # Step 2: Pick c1 randomly
                c1_x = random.uniform(0, self.map.width)
                c1_y = random.uniform(0, self.map.height)

                # Step 3: Sample distance from a normal distribution
                std_dev = min(self.map.width, self.map.height) / 10  # Adjust spread
                d = abs(np.random.normal(0, std_dev))

                # Step 4: Generate c2 at distance d from c1 in a random direction
                theta = random.uniform(0, 2 * np.pi)
                c2_x = c1_x + d * np.cos(theta)
                c2_y = c1_y + d * np.sin(theta)

                # Step 5-10: Check validity and add only one if necessary
                c1_valid = not self.is_collision(c1_x, c1_y)
                c2_valid = not self.is_collision(c2_x, c2_y)

                if c1_valid and not c2_valid:
                    self.samples.append(GraphNode(c1_x, c1_y))
                elif c2_valid and not c1_valid:
                    self.samples.append(GraphNode(c2_x, c2_y))
                elif c1_valid and c2_valid:
                    self.samples.append(GraphNode(c1_x, c1_y))
                    self.samples.append(GraphNode(c2_x, c2_y))

                attempts += 1
            else:
                # Uniform sampling
                x = random.uniform(0, self.map.width)
                y = random.uniform(0, self.map.height)

            if not self.is_collision(x, y):
                self.samples.append(GraphNode(x, y))

            attempts += 1
            
    def generate_default_grid(self):
        """Generate a rectangular grid with spacing equal to neighbour_radius."""
        step = self.neighbour_radius * 0.5
        x = 0.0
        while x <= self.map.width:
            y = 0.0
            while y <= self.map.height:
                if not self.is_collision(x, y):
                    self.samples.append(GraphNode(x, y))
                y += step
            x += step
            
        self.nodes_in_the_grid = len(self.samples)

    def connect_neighbors(self, node: GraphNode):
        """Connects the given node to nearby nodes within neighbour_radius."""
        all_nodes = self.samples + [self.start_node, self.goal_node]
        for other in all_nodes:
            if other is node:
                # Skip self
                continue
            if self.distance(node.get_position(), other.get_position()) <= self.neighbour_radius:
                if not self.is_edge_collision(node.x, node.y, other.x, other.y):
                    cost = self.distance(node.get_position(), other.get_position())
                    node.add_edge(other, cost)

    def a_star(self):
        """A* algorithm to find the shortest path from start to goal."""
        open_set = []
        heapq.heappush(open_set, (0, id(self.start_node), self.start_node))

        g_cost = {}
        f_cost = {}
        self.parent_map = {}

        g_cost[self.start_node] = 0
        f_cost[self.start_node] = self.distance(self.start_node.get_position(), self.goal_node.get_position())

        visited = set()

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current in visited:
                continue
            visited.add(current)

            if current == self.goal_node:
                return

            for neighbour, cost in current.edges.items():
                if neighbour not in g_cost:
                    g_cost[neighbour] = float("inf")
                if neighbour not in f_cost:
                    f_cost[neighbour] = float("inf")

                tentative_g = g_cost[current] + cost
                if tentative_g < g_cost[neighbour]:
                    g_cost[neighbour] = tentative_g
                    f = tentative_g + self.distance(neighbour.get_position(), self.goal_node.get_position())
                    f_cost[neighbour] = f
                    self.parent_map[neighbour] = current
                    heapq.heappush(open_set, (f, id(neighbour), neighbour))

    def is_complete(self):
        """Check if goal node was reached and has a valid parent in parent_map."""
        return self.goal_node in self.parent_map

    def reconstruct_path(self):
        """Reconstructs the shortest path using parent_map from goal to start."""
        self.shortest_path = []
        node = self.goal_node
        while node in self.parent_map:
            self.shortest_path.append(node)
            node = self.parent_map[node]
        self.shortest_path.append(self.start_node)
        self.shortest_path.reverse()
        
    def calculate_shortest_path_cost(self) -> float:
        """Calculate the total cost of the shortest path."""
        if not self.shortest_path or len(self.shortest_path) < 2:
            return float("inf")

        cost = 0.0
        for i in range(1, len(self.shortest_path)):
            cost += self.distance(
                self.shortest_path[i - 1].get_position(),
                self.shortest_path[i].get_position()
            )
        return cost
    
    def clear_nodes(self):
        """
        This overwrites default method as PRM has more attributes.
        We want to end up with clean map and samples. Step should be 0.
        """
        self.samples = []
        self.nodes_in_the_grid = 0
        self.parent_map = {}
        self.shortest_path = []
        self.steps = 0
        self.nodes = []
    
    def reinintialise_start_and_goal(self, start, goal):
        """Clear only the computed shortest path; preserve roadmap structure."""
        logger.info("Clearing the best path.")
        self.start_node = GraphNode(start.x, start.y)
        self.goal_node = GraphNode(goal.x, goal.y)

        if self.nodes_in_the_grid == 0:
            # No roadmap is there, just clear everything and start from scratch
            self.clear_nodes()
        else:
            self.shortest_path = []
            self.parent_map = {}
            self.steps = 2 # Move back to the A* step
            
        # connect start and goal to the roadmap
        self.connect_neighbors(self.start_node)
        self.connect_neighbors(self.goal_node)
