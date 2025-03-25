from abc import ABC, abstractmethod
import math
import time
from core.map import Map
from core.node import TreeNode, GraphNode
from benchmarks.benchmark_manager import BenchmarkManager
from benchmarks.benchmark_result import BenchmarkResult
from core.logger import logger

class Algorithm(ABC):
    def __init__(self,
                 map: Map,
                 step_size: float = 2,
                 benchmark_manager: BenchmarkManager = None,
                 architecture: str = "tree"):
        self.map = map
        self.nodes = []
        self.steps = 0
        self.step_size = step_size
        self.start_time = None
        self.benchmark_manager = benchmark_manager
        self.architecture = architecture
        self.shortest_path = [] # store shortest path starting from start node to goal node
        self.start_node = None
        self.goal_node = None

    @abstractmethod
    def step(self):
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_benchmark()

    def is_complete(self, new_node=None):
        """
        Algorithm is complete if the nearest node is within self.step_size
        distance of the goal.
        
        Args:
            new_node (TreeNode|GraphNode): The new node to check if it is within distance of the goal.
                                           If None, it will check for the last node in the nodes list.

        Returns:
            bool: True if the algorithm is complete, False otherwise.
        """
        if self.map.goal is None:
            return False

        last_node = self.nodes[-1] if new_node is None else new_node
        if last_node is None:
            return False

        distance = self.distance(last_node.get_position(), (self.map.goal.x, self.map.goal.y))
        
        is_within_distance = distance < self.step_size
        goal_achievable = not self.is_edge_collision(last_node.x, last_node.y, self.map.goal.x, self.map.goal.y)
        goal_node_already_in_nodes = self.goal_node in self.nodes
        # Algorithm is complete, append the goal node to the nodes list
        if is_within_distance and goal_achievable and not goal_node_already_in_nodes:
            self.nodes.append(self.goal_node)
            self.goal_node.parent = last_node
            return True
        elif goal_node_already_in_nodes:
            return True

        return False

    def clear_nodes(self):
        # Usually we would like to keep a start node.
        self.nodes = []
        self.start_node = None
        if self.map.start:
            if self.architecture == "tree":
                self.start_node = TreeNode(self.map.start.x, self.map.start.y)
            elif self.architecture == "graph":
                self.start_node = GraphNode(self.map.start.x, self.map.start.y, None)
            if self.start_node is None:
                raise ValueError("Start node is None")

            self.nodes.append(self.start_node)

    def get_nodes(self):
        return self.nodes

    def is_collision(self, x, y):
        """
        Check if the point (x, y) is inside an obstacle. And inside a map.
        """
        # Check if it is inside the map
        if x < 0 or x > self.map.width or y < 0 or y > self.map.height:
            return True

        # Check if it is inside an obstacle
        for ox, oy, w, h in self.map.get_obstacles():
            if ox <= x <= ox + w and oy <= y <= oy + h:
                return True

        return False

    def is_edge_collision(self, x1, y1, x2, y2):
        """
        Check if the line segment (x1, y1) to (x2, y2) intersects with any obstacle.
        
        Returns:
            bool: True if there is a collision, False otherwise
        """
        for ox, oy, w, h in self.map.get_obstacles():
            edges = [
                ((ox, oy), (ox + w, oy)),
                ((ox + w, oy), (ox + w, oy + h)),
                ((ox + w, oy + h), (ox, oy + h)),
                ((ox, oy + h), (ox, oy))
            ]
            for edge_start, edge_end in edges:
                if self.line_intersect(x1, y1, x2, y2, edge_start[0], edge_start[1], edge_end[0], edge_end[1]):
                    return True
        return False

    def line_intersect(self, x1, y1, x2, y2, x3, y3, x4, y4):
        def ccw(ax, ay, bx, by, cx, cy):
            return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)

        return (ccw(x1, y1, x3, y3, x4, y4) != ccw(x2, y2, x3, y3, x4, y4)) and \
               (ccw(x1, y1, x2, y2, x3, y3) != ccw(x1, y1, x2, y2, x4, y4))

    def get_nearest_node(self, sample) -> TreeNode|GraphNode|None:
        if not self.nodes:
            return None

        nearest = None
        min_dist = float('inf')
        for node in self.nodes:
            dist = self.distance(node.get_position(), sample)
            if dist < min_dist:
                nearest = node
                min_dist = dist
        return nearest

    # TODO -> It might not work for PRM
    def reconstruct_path(self):
        logger.info("Reconstructing path...")
        if self.map.goal is None:
            logger.warning("Goal is not set!")
            return
        
        if self.goal_node is None or self.start_node is None:
            logger.warning("Goal or start node is not set!")
            return

        logger.info("Calculating shortest path...")
        self.shortest_path = []

        node = self.goal_node

        while node is not None:
            self.shortest_path.append(node)
            node = node.parent
        self.shortest_path.reverse()

    def distance(self, pos1: tuple, pos2: tuple) -> float:
        if pos1 is None or pos2 is None:
            return float('inf')
        return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

    def calculate_shortest_path_cost(self) -> float:
        """
        Calculate the cost of a shortest path.
        """
        if len(self.shortest_path) == 0:
            logger.warning("Shortest path is empty!")
            return float('inf')

        # Validate that the first node is the start node
        if self.shortest_path[0] != self.start_node:
            print(f"Start node: {self.start_node.get_position()}")
            print(f"First node: {self.shortest_path[0].get_position()}")
            logger.error("Shortest path does not start from the start node!")
            return float('inf')
        
        # Validate that the last node is the goal node
        if self.shortest_path[-1] != self.goal_node:
            print(f"Goal node: {self.goal_node.get_position()}")
            print(f"Last node: {self.shortest_path[-1].get_position()}")
            logger.error("Shortest path does not end with the goal node!")
            return float('inf')

        cost = 0.0
        for node in self.shortest_path:
            if node.parent is not None:
                cost += self.distance(node.get_position(), node.parent.get_position())

        return cost

    def start_benchmark(self):
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_time = time.time()
            # logger.info(f"Benchmark started for {self.__class__.__name__}")

    def finalize_benchmark(self):
        if self.benchmark_manager is None:
            logger.warning(f"No benchmark specified!")
            return

        if self.start_time is None:
            logger.warning(f"Time is not running!")
            return

        execution_time = time.time() - self.start_time

        result = BenchmarkResult(
            algorithm_name=self.__class__.__name__,
            steps=self.steps,
            execution_time=execution_time,
            start_point=self.map.start,
            goal_point=self.map.goal,
            step_size=self.step_size,
            path_length=self.calculate_shortest_path_cost(),
            shortest_path=self.shortest_path
        )

        self.benchmark_manager.add_result(result)
        #self.benchmark_manager.print_results()
        # logger.info(f"Benchmark completed for {self.__class__.__name__}")
