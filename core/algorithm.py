from abc import ABC, abstractmethod
import math
from core.map import Map
import time
from benchmarks.benchmark_manager import BenchmarkManager
from benchmarks.benchmark_result import BenchmarkResult

class Algorithm(ABC):
    def __init__(self, map: Map, benchmark_manager: BenchmarkManager = None):
        self.map = map
        self.nodes = []
        self.steps = 0
        self.start_time = None
        self.benchmark_manager = benchmark_manager

    @abstractmethod
    def step(self):
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_benchmark()  # Start benchmark when first step is called

    @abstractmethod
    def is_complete(self):
        pass

    @abstractmethod
    def clear_nodes(self):
        # Usually we would like to keep the start node.
        if self.map.start:
            self.nodes = [self.map.start]

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
            x1, y1 = self.nodes[i - 1]
            x2, y2 = self.nodes[i]
            length += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return length

    # Start benchmark automatically when algorithm starts
    def start_benchmark(self):
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_time = time.time()
            print(f"Benchmark started for {self.__class__.__name__}")

    # Finalize benchmark automatically when goal is reached
    def finalize_benchmark(self):
        if self.benchmark_manager is None:
            print(f"No benchmark specified!")
            return

        if self.start_time is None:
            print(f"Time is not running!")
            return

        execution_time = time.time() - self.start_time
        path_length = self.compute_path_length()

        result = BenchmarkResult(
            algorithm_name=self.__class__.__name__,
            path_length=path_length,
            steps=self.steps,
            execution_time=execution_time
        )

        self.benchmark_manager.add_result(result)
        self.benchmark_manager.print_results()
        print(f"Benchmark completed for {self.__class__.__name__}")
