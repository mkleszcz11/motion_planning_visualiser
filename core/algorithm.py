from abc import ABC, abstractmethod
import math
import time
from core.map import Map
from core.node import Node
from benchmarks.benchmark_manager import BenchmarkManager
from benchmarks.benchmark_result import BenchmarkResult
from core.logger import logger

class Algorithm(ABC):
    def __init__(self,
                 map: Map,
                 step_size: float = 2,
                 benchmark_manager: BenchmarkManager = None):
        self.map = map
        self.nodes = []
        self.steps = 0
        self.step_size = step_size
        self.start_time = None
        self.benchmark_manager = benchmark_manager

    @abstractmethod
    def step(self):
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_benchmark()

    def is_complete(self):
        """
        Algorithm is complete if the nearest node is within self.step_size
        distance of the goal.
        """
        if self.map.goal is None:
            return False

        last_node = self.get_nearest_node((self.map.goal.x, self.map.goal.y))
        if last_node is None:
            return False

        distance = self.distance(last_node.get_position(), (self.map.goal.x, self.map.goal.y))
        return distance < self.step_size


    def clear_nodes(self):
        # Usually we would like to keep a start node.
        self.nodes = []
        if self.map.start:
            start_node = Node(self.map.start.x, self.map.start.y)
            self.nodes.append(start_node)

    def get_nodes(self):
        return self.nodes

    def is_collision(self, x, y):
        for ox, oy, w, h in self.map.get_obstacles():
            if ox <= x <= ox + w and oy <= y <= oy + h:
                return True
        return False

    def is_edge_collision(self, x1, y1, x2, y2):
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

    def compute_path_length(self):
        length = 0
        for i in range(1, len(self.nodes)):
            x1, y1 = self.nodes[i - 1].get_position()
            x2, y2 = self.nodes[i].get_position()
            length += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return length

    def get_nearest_node(self, sample):
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
        # TODO -> It might not work for PRM
        logger.info("Reconstructing path...")
        if self.map.goal is None:
            return

        logger.info("Calculating shortest path...")
        self.shortest_path = []
        node = self.get_nearest_node((self.map.goal.x, self.map.goal.y))
        if node is None: # ADDED THIS
            return

        while node is not None:
            self.shortest_path.append(node)
            node = node.parent
        self.shortest_path.reverse()

    def distance(self, pos1, pos2):
        if pos1 is None or pos2 is None:
            return float('inf')
        return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

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
        path_length = self.compute_path_length()

        result = BenchmarkResult(
            algorithm_name=self.__class__.__name__,
            path_length=path_length,
            steps=self.steps,
            execution_time=execution_time,
            start_point=self.map.start,
            goal_point=self.map.goal,
            step_size=self.step_size,
            path=self.shortest_path
        )

        self.benchmark_manager.add_result(result)
        #self.benchmark_manager.print_results()
        # logger.info(f"Benchmark completed for {self.__class__.__name__}")
