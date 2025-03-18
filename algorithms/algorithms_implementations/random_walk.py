import random
import math
from core.algorithm import Algorithm

STEP_SIZE = 5

class RandomWalkAlgorithm(Algorithm):
    def __init__(self, map, benchmark_manager=None):
        super().__init__(map, benchmark_manager)
        if map.start:
            self.nodes.append(map.start)
            
    def clear_nodes(self):
        super().clear_nodes()

    def step(self):
        if self.start_time is None and self.benchmark_manager is not None:
            self.start_benchmark()
        
        if self.is_complete():
            if self.map.goal not in self.nodes:
                self.nodes.append(self.map.goal)
                self.finalize_benchmark()
            return

        if not self.nodes:
            return

        last_node = self.nodes[-1]
        new_x = last_node[0] + random.uniform(-STEP_SIZE, STEP_SIZE)
        new_y = last_node[1] + random.uniform(-STEP_SIZE, STEP_SIZE)

        new_x = max(0, min(self.map.width, new_x))
        new_y = max(0, min(self.map.height, new_y))

        if not self.is_collision(new_x, new_y) and not self.is_edge_collision(last_node[0], last_node[1], new_x, new_y):
            self.nodes.append((new_x, new_y))
            self.steps += 1

            if self.is_complete():
                self.nodes.append(self.map.goal)
                self.finalize_benchmark()  # ✅ Trigger benchmarking in parent

    def is_complete(self):
        if not self.map.goal or not self.nodes:
            return False
        last_node = self.nodes[-1]
        distance = math.sqrt((last_node[0] - self.map.goal[0])**2 +
                             (last_node[1] - self.map.goal[1])**2)
        return distance < STEP_SIZE
